#!/usr/bin/env python3
"""Live G1 Regular-mode locomotion <-> stationary XR arm handover.

THIS PROGRAM PUBLISHES REAL ARM COMMANDS TO ``rt/arm_sdk``.

Architecture
------------
LOCOMOTION_READY
    Unitree Regular mode owns legs, waist, balance, and arms.
    The R3 operator may move the robot.

WAITING_FOR_STOP
    R3 sticks must be neutral and LowState must indicate that the lower body
    has settled for the configured hold time.

XR_ALIGNMENT
    The VR operator matches the current robot arm pose. One common XYZ
    translation is registered for both wrists.

ARM_RAMP_UP
    Live head-yaw-relative XR targets are solved by IK while arm ownership is
    ramped smoothly from zero to ``--teleop-weight``.

XR_ACTIVE
    The robot remains stationary and the XR operator controls both arms.

RETURN_ARMS_HOME
    XR is frozen and the commanded 14-joint arm target is moved smoothly to
    the Regular-mode handover pose q=0.

ARM_RAMP_DOWN
    Ownership is ramped to zero, returning the arms to the stock controller.

Safety model
------------
- R3 movement during XR control freezes the XR command and enters a fault hold.
- Sustained lower-body movement during XR control also enters a fault hold.
- Invalid XR frames freeze the command and enter a fault hold.
- The current TeleVuer hand stream may retain its last valid hand sample when
  hands are hidden. Therefore, hand hiding is NOT treated as a guaranteed
  emergency-stop mechanism.
- ``c`` is a mode/handover command, not an emergency stop.
- The R3 operator must retain immediate access to the robot's verified damping
  or emergency-stop procedure.

Keys
----
c : request XR mode, cancel a pending request, or request handback
p : print a detailed status snapshot
q : perform a controlled handback and exit

Use only the locally served Quest page:
    https://<THINKCENTRE-IP>:8012/?ws=wss://<THINKCENTRE-IP>:8012
"""

from __future__ import annotations

import argparse
import logging
import math
import signal
import struct
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening

CURRENT_DIR = Path(__file__).resolve().parent
PARENT_DIR = CURRENT_DIR.parent
if str(PARENT_DIR) not in sys.path:
    sys.path.append(str(PARENT_DIR))

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_ as HgLowCmd,
    LowState_ as HgLowState,
)
from unitree_sdk2py.utils.crc import CRC

from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

# Reuse only the helpers already validated in the successful arm-only tests.
from teleop_arm_only_direct_headyaw_registered_clutch import (
    apply_translation_bias,
    compute_shared_translation_bias,
    current_wrist_poses_from_fk,
    format_vector,
    pose_is_valid,
    rate_limit_pose,
    raw_head_pose,
    robot_yaw_deg,
    tracking_frame_valid,
    update_guarded_pose,
)


TOGGLE_REQUESTED = False
SNAPSHOT_REQUESTED = False
QUIT_REQUESTED = False
FORCE_STOP = threading.Event()
SIGNAL_COUNT = 0


ARM_INDICES = tuple(range(15, 22)) + tuple(range(22, 29))
LEG_WAIST_INDICES = tuple(range(15))
WRIST_INDICES = {19, 20, 21, 26, 27, 28}
WEAK_INDICES = {4, 10, 15, 16, 17, 18, 22, 23, 24, 25}
WEIGHT_INDEX = 29
MOTOR_COUNT = 35


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_locomotion_xr_live_v2")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)-8s [HandoverLive] %(message)s",
            "%H:%M:%S",
        )
    )
    logger.addHandler(handler)
    logger.propagate = False
    return logger


LOG = configure_logging()


class State(Enum):
    LOCOMOTION_READY = auto()
    WAITING_FOR_STOP = auto()
    XR_ALIGNMENT = auto()
    ARM_RAMP_UP = auto()
    XR_ACTIVE = auto()
    SAFETY_FAULT_HOLD = auto()
    RETURN_ARMS_HOME = auto()
    ARM_RAMP_DOWN = auto()


@dataclass
class RemoteState:
    valid: bool = False
    keys: int = 0
    lx: float = math.nan
    ly: float = math.nan
    rx: float = math.nan
    ry: float = math.nan

    @property
    def max_abs_axis(self) -> float:
        axes = np.asarray(
            [self.lx, self.ly, self.rx, self.ry],
            dtype=float,
        )
        if not np.isfinite(axes).all():
            return math.nan
        return float(np.max(np.abs(axes)))


@dataclass
class MotionMetrics:
    valid: bool = False
    lowstate_age_s: float = math.inf
    leg_waist_dq_max_rps: float = math.nan
    leg_waist_dq_rms_rps: float = math.nan
    gyro_z_rps: float = math.nan
    remote: Optional[RemoteState] = None

    def __post_init__(self) -> None:
        if self.remote is None:
            self.remote = RemoteState()


@dataclass
class XrCommandState:
    last_good_left: Optional[np.ndarray] = None
    last_good_right: Optional[np.ndarray] = None
    left_pending: Optional[np.ndarray] = None
    right_pending: Optional[np.ndarray] = None
    left_pending_count: int = 0
    right_pending_count: int = 0
    guarded_left: Optional[np.ndarray] = None
    guarded_right: Optional[np.ndarray] = None
    last_left_command: Optional[np.ndarray] = None
    last_right_command: Optional[np.ndarray] = None
    last_command_time: float = 0.0
    guard_active: bool = False

    def reset(
        self,
        registered_left: np.ndarray,
        registered_right: np.ndarray,
        robot_left: np.ndarray,
        robot_right: np.ndarray,
    ) -> None:
        self.last_good_left = registered_left.copy()
        self.last_good_right = registered_right.copy()
        self.left_pending = None
        self.right_pending = None
        self.left_pending_count = 0
        self.right_pending_count = 0
        self.guarded_left = registered_left.copy()
        self.guarded_right = registered_right.copy()

        # The first commanded Cartesian pose is the robot's measured pose.
        # This avoids a target discontinuity when ownership begins ramping up.
        self.last_left_command = robot_left.copy()
        self.last_right_command = robot_right.copy()
        self.last_command_time = time.monotonic()
        self.guard_active = False


class LowStateCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._message: Optional[HgLowState] = None
        self._received_monotonic: Optional[float] = None
        self._count = 0

    def callback(self, message: HgLowState) -> None:
        with self._lock:
            self._message = message
            self._received_monotonic = time.monotonic()
            self._count += 1

    def snapshot(
        self,
    ) -> tuple[Optional[HgLowState], float, int]:
        with self._lock:
            message = self._message
            received = self._received_monotonic
            count = self._count

        age = (
            time.monotonic() - received
            if received is not None
            else math.inf
        )
        return message, age, count


class SafeArmSdkController:
    """Minimal G1 arm_sdk publisher with explicit ownership control.

    Unlike the stock G1_29_ArmController motion-mode implementation, this
    controller starts at ownership weight zero, seeds its arm target from
    the measured robot pose before any non-zero weight is permitted, and
    accumulates its rate-limited user command from the previously published
    user command.
    """

    def __init__(
        self,
        cache: LowStateCache,
        initial_lowstate: HgLowState,
        *,
        joint_velocity_limit_rps: float,
        publish_hz: float = 250.0,
    ) -> None:
        self._cache = cache
        self._joint_velocity_limit_rps = float(
            joint_velocity_limit_rps
        )
        self._publish_dt = 1.0 / float(publish_hz)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._weight = 0.0
        self._write_count = 0
        self._last_write_exception: Optional[Exception] = None

        initial_q_all = np.asarray(
            [
                float(initial_lowstate.motor_state[i].q)
                for i in range(MOTOR_COUNT)
            ],
            dtype=float,
        )
        if initial_q_all.shape != (MOTOR_COUNT,):
            raise RuntimeError("Unexpected G1 LowState motor count")
        if not np.isfinite(initial_q_all).all():
            raise RuntimeError("Initial G1 motor positions contain NaN/Inf")

        self._q_target = initial_q_all[list(ARM_INDICES)].copy()
        self._tau_target = np.zeros(14, dtype=float)
        self._last_sent_q = self._q_target.copy()

        self._publisher = ChannelPublisher("rt/arm_sdk", HgLowCmd)
        self._publisher.Init()

        self._crc = CRC()
        self._msg = unitree_hg_msg_dds__LowCmd_()
        self._msg.mode_pr = 0
        self._msg.mode_machine = int(initial_lowstate.mode_machine)

        for index in range(MOTOR_COUNT):
            command = self._msg.motor_cmd[index]
            command.mode = 1

            if index in ARM_INDICES:
                if index in WRIST_INDICES:
                    command.kp = 40.0
                    command.kd = 1.5
                else:
                    command.kp = 80.0
                    command.kd = 3.0
            else:
                if index in WEAK_INDICES:
                    command.kp = 80.0
                    command.kd = 3.0
                else:
                    command.kp = 300.0
                    command.kd = 3.0

            command.q = float(initial_q_all[index])
            command.dq = 0.0
            command.tau = 0.0

        # Critical invariant: publish weight zero from the very first frame.
        self._msg.motor_cmd[WEIGHT_INDEX].q = 0.0

        self._thread = threading.Thread(
            target=self._publish_loop,
            name="g1-arm-sdk-safe-publisher",
            daemon=True,
        )
        self._thread.start()

    def _publish_loop(self) -> None:
        while not self._stop.is_set():
            loop_start = time.monotonic()

            with self._lock:
                target_q = self._q_target.copy()
                target_tau = self._tau_target.copy()
                weight = float(self._weight)

            # Rate-limit relative to the previously published USER command,
            # not relative to the measured joint state.
            #
            # The V1 controller used measured q as the limiter reference.
            # At partial arm ownership, the stock controller can keep measured
            # q close to its own pose, causing every user command to remain only
            # one tiny step away from measured q. That makes 0.20/0.50 weight
            # look almost unresponsive. Accumulating from the last published
            # user command preserves the requested velocity limit while still
            # allowing the user target to progress toward the IK solution.
            reference_q = self._last_sent_q.copy()

            max_step = (
                self._joint_velocity_limit_rps * self._publish_dt
            )
            delta = target_q - reference_q
            max_delta = float(np.max(np.abs(delta)))

            if max_delta > max_step and max_delta > 1e-12:
                commanded_q = reference_q + delta * (
                    max_step / max_delta
                )
            else:
                commanded_q = target_q

            for local_index, motor_index in enumerate(ARM_INDICES):
                command = self._msg.motor_cmd[motor_index]
                command.q = float(commanded_q[local_index])
                command.dq = 0.0
                command.tau = float(target_tau[local_index])

            self._msg.motor_cmd[WEIGHT_INDEX].q = float(
                np.clip(weight, 0.0, 1.0)
            )

            try:
                self._msg.crc = self._crc.Crc(self._msg)
                self._publisher.Write(self._msg)
                self._write_count += 1
                self._last_sent_q = commanded_q.copy()
            except Exception as exc:
                self._last_write_exception = exc

            elapsed = time.monotonic() - loop_start
            self._stop.wait(max(0.0, self._publish_dt - elapsed))

    def set_target(
        self,
        q_target: np.ndarray,
        tau_target: Optional[np.ndarray] = None,
    ) -> None:
        q = np.asarray(q_target, dtype=float).reshape(-1)
        tau = (
            np.zeros(14, dtype=float)
            if tau_target is None
            else np.asarray(tau_target, dtype=float).reshape(-1)
        )

        if q.shape != (14,) or tau.shape != (14,):
            raise ValueError("Expected 14 arm positions and torques")
        if not np.isfinite(q).all() or not np.isfinite(tau).all():
            raise ValueError("Arm target contains NaN/Inf")

        with self._lock:
            self._q_target = q.copy()
            self._tau_target = tau.copy()

    def get_target_q(self) -> np.ndarray:
        with self._lock:
            return self._q_target.copy()

    def get_last_sent_q(self) -> np.ndarray:
        # Only the publisher thread writes this array; copying is atomic enough
        # for status diagnostics and never affects control.
        return self._last_sent_q.copy()

    def set_weight(self, weight: float) -> None:
        if not math.isfinite(weight):
            raise ValueError("Arm ownership weight is not finite")

        with self._lock:
            self._weight = float(np.clip(weight, 0.0, 1.0))

    def get_weight(self) -> float:
        with self._lock:
            return float(self._weight)

    @property
    def write_count(self) -> int:
        return self._write_count

    @property
    def last_write_exception(self) -> Optional[Exception]:
        return self._last_write_exception

    def release_weight_only(self, duration_s: float = 1.0) -> None:
        start_weight = self.get_weight()
        start = time.monotonic()

        while time.monotonic() - start < duration_s:
            progress = (time.monotonic() - start) / duration_s
            self.set_weight(
                start_weight * (1.0 - smoothstep(progress))
            )
            time.sleep(0.01)

        self.set_weight(0.0)
        time.sleep(0.10)

    def close(self) -> None:
        self.set_weight(0.0)
        time.sleep(0.10)
        self._stop.set()
        self._thread.join(timeout=1.0)

        try:
            self._publisher.Close()
        except Exception:
            pass


def smoothstep(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def keyboard_press(key: str) -> None:
    global TOGGLE_REQUESTED, SNAPSHOT_REQUESTED, QUIT_REQUESTED

    if key == "c":
        TOGGLE_REQUESTED = True
        LOG.info("[c] mode toggle requested.")
    elif key == "p":
        SNAPSHOT_REQUESTED = True
    elif key == "q":
        QUIT_REQUESTED = True
        LOG.info("[q] controlled exit requested.")
    else:
        LOG.warning("Unknown key %r. Use c, p, or q.", key)


def signal_stop(signum: int, frame: object) -> None:
    del signum, frame
    global SIGNAL_COUNT, QUIT_REQUESTED

    SIGNAL_COUNT += 1
    if SIGNAL_COUNT == 1:
        QUIT_REQUESTED = True
        LOG.warning(
            "Ctrl+C received: requesting controlled handback. "
            "Press Ctrl+C again only to force termination."
        )
    else:
        LOG.error("Second Ctrl+C received: forcing termination.")
        FORCE_STOP.set()
        try:
            stop_listening()
        except Exception:
            pass


def parse_embedded_remote(lowstate: HgLowState) -> RemoteState:
    """Decode the R3 packet embedded in hg LowState."""
    try:
        data = bytes(lowstate.wireless_remote)
        if len(data) < 24:
            return RemoteState()

        keys = struct.unpack_from("<H", data, 2)[0]
        lx = struct.unpack_from("<f", data, 4)[0]
        rx = struct.unpack_from("<f", data, 8)[0]
        ry = struct.unpack_from("<f", data, 12)[0]
        ly = struct.unpack_from("<f", data, 20)[0]

        axes = np.asarray([lx, ly, rx, ry], dtype=float)
        if not np.isfinite(axes).all():
            return RemoteState()
        if np.max(np.abs(axes)) > 1.5:
            return RemoteState()

        return RemoteState(
            valid=True,
            keys=int(keys),
            lx=float(lx),
            ly=float(ly),
            rx=float(rx),
            ry=float(ry),
        )
    except Exception:
        return RemoteState()


def extract_motion_metrics(
    lowstate: Optional[HgLowState],
    age_s: float,
    freshness_s: float,
) -> MotionMetrics:
    if lowstate is None or age_s > freshness_s:
        return MotionMetrics(lowstate_age_s=age_s)

    try:
        leg_waist_dq = np.asarray(
            [
                float(lowstate.motor_state[i].dq)
                for i in LEG_WAIST_INDICES
            ],
            dtype=float,
        )

        if (
            leg_waist_dq.shape != (15,)
            or not np.isfinite(leg_waist_dq).all()
        ):
            return MotionMetrics(lowstate_age_s=age_s)

        gyro = np.asarray(
            lowstate.imu_state.gyroscope,
            dtype=float,
        ).reshape(-1)
        if gyro.size < 3 or not np.isfinite(gyro[:3]).all():
            return MotionMetrics(lowstate_age_s=age_s)

        remote = parse_embedded_remote(lowstate)

        return MotionMetrics(
            valid=remote.valid,
            lowstate_age_s=age_s,
            leg_waist_dq_max_rps=float(
                np.max(np.abs(leg_waist_dq))
            ),
            leg_waist_dq_rms_rps=float(
                np.sqrt(np.mean(np.square(leg_waist_dq)))
            ),
            gyro_z_rps=float(gyro[2]),
            remote=remote,
        )
    except Exception:
        return MotionMetrics(lowstate_age_s=age_s)


def current_arm_q_dq(
    lowstate: HgLowState,
) -> tuple[np.ndarray, np.ndarray]:
    q = np.asarray(
        [float(lowstate.motor_state[i].q) for i in ARM_INDICES],
        dtype=float,
    )
    dq = np.asarray(
        [float(lowstate.motor_state[i].dq) for i in ARM_INDICES],
        dtype=float,
    )

    if (
        q.shape != (14,)
        or dq.shape != (14,)
        or not np.isfinite(q).all()
        or not np.isfinite(dq).all()
    ):
        raise RuntimeError("Invalid measured arm state")

    return q, dq


def tracking_ready(
    tv_wrapper: TeleVuerWrapper,
    tele_data: object,
) -> tuple[bool, str]:
    valid, reason = tracking_frame_valid(tv_wrapper, tele_data)
    if not valid:
        return False, reason
    if raw_head_pose(tv_wrapper) is None:
        return False, "raw headset pose unavailable"
    return True, "ok"


def stop_gate_instant(
    metrics: MotionMetrics,
    args: argparse.Namespace,
) -> bool:
    if not metrics.valid or metrics.remote is None:
        return False

    return (
        metrics.remote.max_abs_axis <= args.r3_deadband
        and metrics.leg_waist_dq_rms_rps
        <= args.stop_dq_rms_rps
        and metrics.leg_waist_dq_max_rps
        <= args.stop_dq_hard_max_rps
        and abs(metrics.gyro_z_rps)
        <= args.stop_yaw_rate_rps
    )


def update_xr_command(
    *,
    arm_ik: G1_29_ArmIK,
    arm_ctrl: SafeArmSdkController,
    xr_state: XrCommandState,
    tele_data: object,
    translation_bias: np.ndarray,
    current_q: np.ndarray,
    current_dq: np.ndarray,
    args: argparse.Namespace,
) -> tuple[bool, str]:
    live_left = np.asarray(
        tele_data.left_wrist_pose,
        dtype=float,
    ).copy()
    live_right = np.asarray(
        tele_data.right_wrist_pose,
        dtype=float,
    ).copy()

    registered_left = apply_translation_bias(
        live_left,
        translation_bias,
    )
    registered_right = apply_translation_bias(
        live_right,
        translation_bias,
    )

    (
        guarded_left,
        last_good_left,
        left_pending,
        left_pending_count,
        left_rejected,
        _,
    ) = update_guarded_pose(
        registered_left,
        xr_state.last_good_left,
        xr_state.left_pending,
        xr_state.left_pending_count,
        args.tracking_jump_m,
        args.tracking_jump_deg,
        args.tracking_reacquire_frames,
    )

    (
        guarded_right,
        last_good_right,
        right_pending,
        right_pending_count,
        right_rejected,
        _,
    ) = update_guarded_pose(
        registered_right,
        xr_state.last_good_right,
        xr_state.right_pending,
        xr_state.right_pending_count,
        args.tracking_jump_m,
        args.tracking_jump_deg,
        args.tracking_reacquire_frames,
    )

    xr_state.guarded_left = guarded_left
    xr_state.guarded_right = guarded_right
    xr_state.last_good_left = last_good_left
    xr_state.last_good_right = last_good_right
    xr_state.left_pending = left_pending
    xr_state.right_pending = right_pending
    xr_state.left_pending_count = left_pending_count
    xr_state.right_pending_count = right_pending_count
    xr_state.guard_active = left_rejected or right_rejected

    now = time.monotonic()
    command_dt = float(
        np.clip(now - xr_state.last_command_time, 0.005, 0.10)
    )
    xr_state.last_command_time = now

    left_command = rate_limit_pose(
        xr_state.last_left_command,
        guarded_left,
        command_dt,
        args.max_wrist_speed,
        args.max_wrist_rotation_speed_deg,
    )
    right_command = rate_limit_pose(
        xr_state.last_right_command,
        guarded_right,
        command_dt,
        args.max_wrist_speed,
        args.max_wrist_rotation_speed_deg,
    )

    xr_state.last_left_command = left_command.copy()
    xr_state.last_right_command = right_command.copy()

    try:
        solution_q, solution_tau = arm_ik.solve_ik(
            left_command,
            right_command,
            current_q,
            current_dq,
        )
    except Exception as exc:
        return False, f"IK exception: {exc}"

    solution_q = np.asarray(solution_q, dtype=float).reshape(-1)
    solution_tau = np.asarray(solution_tau, dtype=float).reshape(-1)

    if (
        solution_q.shape != (14,)
        or solution_tau.shape != (14,)
        or not np.isfinite(solution_q).all()
        or not np.isfinite(solution_tau).all()
    ):
        return False, "IK returned an invalid 14-joint command"

    previous_target = arm_ctrl.get_target_q()
    ik_target_jump = float(
        np.max(np.abs(solution_q - previous_target))
    )
    if ik_target_jump > args.max_ik_target_jump_rad:
        return (
            False,
            "IK target discontinuity "
            f"{ik_target_jump:.3f} rad exceeds "
            f"{args.max_ik_target_jump_rad:.3f} rad",
        )

    arm_ctrl.set_target(solution_q, solution_tau)
    return True, "ok"


def begin_return(
    state_data: dict[str, object],
    arm_ctrl: SafeArmSdkController,
    now: float,
) -> None:
    state_data["return_start_q"] = arm_ctrl.get_target_q()
    state_data["return_start_weight"] = arm_ctrl.get_weight()
    state_data["state_started"] = now


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Live G1 Regular-mode locomotion/stationary-XR handover."
        )
    )
    parser.add_argument(
        "--enable-live-arm-sdk",
        action="store_true",
        help=(
            "Required acknowledgement: publish real commands to rt/arm_sdk."
        ),
    )
    parser.add_argument(
        "--network-interface",
        type=str,
        required=True,
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=30.0,
    )
    parser.add_argument(
        "--status-hz",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--lowstate-fresh-s",
        type=float,
        default=0.10,
    )

    parser.add_argument(
        "--teleop-weight",
        type=float,
        default=0.20,
        help=(
            "Maximum arm ownership. Use 0.20 for the first suspended test, "
            "then 0.50, and only later 1.0."
        ),
    )
    parser.add_argument(
        "--arm-ramp-up-s",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--return-arms-s",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--return-settle-timeout-s",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--return-tolerance-rad",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--arm-ramp-down-s",
        type=float,
        default=2.5,
    )
    parser.add_argument(
        "--joint-target-speed-rps",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--max-ik-target-jump-rad",
        type=float,
        default=0.35,
        help=(
            "Reject a sudden IK branch change relative to the previous "
            "14-joint target."
        ),
    )

    parser.add_argument(
        "--r3-deadband",
        type=float,
        default=0.12,
    )
    parser.add_argument(
        "--stop-dq-rms-rps",
        type=float,
        default=0.18,
    )
    parser.add_argument(
        "--stop-dq-hard-max-rps",
        type=float,
        default=0.75,
    )
    parser.add_argument(
        "--stop-yaw-rate-rps",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--stop-hold-s",
        type=float,
        default=1.5,
    )

    parser.add_argument(
        "--start-align-m",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--max-registration-shift-m",
        type=float,
        default=0.30,
    )
    parser.add_argument(
        "--stable-align-frames",
        type=int,
        default=20,
    )

    parser.add_argument(
        "--tracking-jump-m",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--tracking-jump-deg",
        type=float,
        default=40.0,
    )
    parser.add_argument(
        "--tracking-reacquire-frames",
        type=int,
        default=8,
    )
    parser.add_argument(
        "--max-wrist-speed",
        type=float,
        default=0.12,
    )
    parser.add_argument(
        "--max-wrist-rotation-speed-deg",
        type=float,
        default=60.0,
    )

    parser.add_argument(
        "--active-r3-fault",
        type=float,
        default=0.18,
    )
    parser.add_argument(
        "--active-r3-fault-frames",
        type=int,
        default=2,
    )
    parser.add_argument(
        "--active-dq-rms-fault-rps",
        type=float,
        default=0.45,
    )
    parser.add_argument(
        "--active-dq-max-fault-rps",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--active-yaw-fault-rps",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--active-body-fault-frames",
        type=int,
        default=8,
    )
    parser.add_argument(
        "--tracking-fault-frames",
        type=int,
        default=3,
    )
    parser.add_argument(
        "--lowstate-fault-frames",
        type=int,
        default=3,
    )
    return parser


def main() -> int:
    global TOGGLE_REQUESTED, SNAPSHOT_REQUESTED, QUIT_REQUESTED

    args = build_parser().parse_args()

    if not args.enable_live_arm_sdk:
        raise SystemExit(
            "Refusing to publish. Re-run with --enable-live-arm-sdk only "
            "after suspending the robot and preparing the R3 safety operator."
        )

    positive_names = (
        "frequency",
        "status_hz",
        "lowstate_fresh_s",
        "arm_ramp_up_s",
        "return_arms_s",
        "return_settle_timeout_s",
        "return_tolerance_rad",
        "arm_ramp_down_s",
        "joint_target_speed_rps",
        "max_ik_target_jump_rad",
        "r3_deadband",
        "stop_dq_rms_rps",
        "stop_dq_hard_max_rps",
        "stop_yaw_rate_rps",
        "stop_hold_s",
        "start_align_m",
        "max_registration_shift_m",
        "tracking_jump_m",
        "tracking_jump_deg",
        "max_wrist_speed",
        "max_wrist_rotation_speed_deg",
    )
    for name in positive_names:
        if getattr(args, name) <= 0:
            raise ValueError(
                f"--{name.replace('_', '-')} must be positive"
            )

    if not 0.0 < args.teleop_weight <= 1.0:
        raise ValueError("--teleop-weight must be in (0, 1]")

    integer_names = (
        "stable_align_frames",
        "tracking_reacquire_frames",
        "active_r3_fault_frames",
        "active_body_fault_frames",
        "tracking_fault_frames",
        "lowstate_fault_frames",
    )
    for name in integer_names:
        if getattr(args, name) < 1:
            raise ValueError(
                f"--{name.replace('_', '-')} must be at least 1"
            )

    signal.signal(signal.SIGINT, signal_stop)
    signal.signal(signal.SIGTERM, signal_stop)

    keyboard_thread: Optional[threading.Thread] = None
    lowstate_subscriber: Optional[ChannelSubscriber] = None
    tv_wrapper: Optional[TeleVuerWrapper] = None
    arm_ctrl: Optional[SafeArmSdkController] = None

    graceful_release_completed = False

    try:
        ChannelFactoryInitialize(
            0,
            networkInterface=args.network_interface,
        )

        cache = LowStateCache()
        lowstate_subscriber = ChannelSubscriber(
            "rt/lowstate",
            HgLowState,
        )
        lowstate_subscriber.Init(cache.callback, 1)

        LOG.info("Waiting for fresh G1 LowState...")
        initial_lowstate: Optional[HgLowState] = None
        deadline = time.monotonic() + 8.0

        while time.monotonic() < deadline:
            sample, age, _ = cache.snapshot()
            if sample is not None and age <= args.lowstate_fresh_s:
                initial_lowstate = sample
                break
            time.sleep(0.02)

        if initial_lowstate is None:
            raise RuntimeError(
                "No fresh rt/lowstate sample received on the selected "
                "network interface"
            )

        keyboard_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={
                "on_press": keyboard_press,
                "until": None,
                "sequential": False,
            },
            daemon=True,
        )
        keyboard_thread.start()

        LOG.info("G1_LOCOMOTION_XR_HANDOVER_LIVE_V2")
        LOG.warning(
            "LIVE ARM COMMANDS ENABLED: publishing rt/arm_sdk."
        )
        LOG.info(
            "V2 limiter active: user arm commands accumulate from the "
            "previously published user command."
        )
        LOG.info(
            "Interface=%s | maximum ownership weight=%.2f | "
            "accumulated joint target speed<=%.2f rad/s | "
            "IK jump<=%.2f rad",
            args.network_interface,
            args.teleop_weight,
            args.joint_target_speed_rps,
            args.max_ik_target_jump_rad,
        )
        LOG.info(
            "Stop gate: R3<=%.2f, lower-body RMS<=%.2f, max<=%.2f, "
            "|yaw|<=%.2f for %.1f s.",
            args.r3_deadband,
            args.stop_dq_rms_rps,
            args.stop_dq_hard_max_rps,
            args.stop_yaw_rate_rps,
            args.stop_hold_s,
        )
        LOG.info(
            "Open only: "
            "https://<THINKCENTRE-IP>:8012/?ws=wss://<THINKCENTRE-IP>:8012"
        )

        tv_wrapper = TeleVuerWrapper(
            use_hand_tracking=True,
            binocular=False,
            img_shape=(480, 640),
            display_mode="pass-through",
            zmq=False,
            webrtc=False,
            webrtc_url=None,
            arm_reference_mode="head_yaw",
        )
        arm_ik = G1_29_ArmIK()

        arm_ctrl = SafeArmSdkController(
            cache,
            initial_lowstate,
            joint_velocity_limit_rps=args.joint_target_speed_rps,
        )
        arm_ctrl.set_weight(0.0)

        state = State.LOCOMOTION_READY
        state_started = time.monotonic()
        state_data: dict[str, object] = {
            "state_started": state_started,
            "return_start_q": np.zeros(14, dtype=float),
            "return_start_weight": 0.0,
        }

        stop_quiet_since: Optional[float] = None
        align_stable_count = 0
        align_bias_sum = np.zeros(3, dtype=float)
        align_bias_samples = 0
        accepted_bias = np.zeros(3, dtype=float)
        xr_state = XrCommandState()

        remote_fault_count = 0
        body_fault_count = 0
        tracking_fault_count = 0
        lowstate_fault_count = 0
        safety_fault_reason = ""
        shutdown_after_release = False
        last_status = 0.0
        last_tracking_reason: Optional[str] = None
        last_guard_state = False

        LOG.info(
            "STATE LOCOMOTION_READY. Arm ownership=0. "
            "The R3 operator may move normally."
        )
        LOG.warning(
            "The hidden-hand watchdog is best-effort only. "
            "Use verbal coordination, [c], and the R3 safety operator."
        )

        while not FORCE_STOP.is_set():
            loop_start = time.monotonic()
            now = loop_start

            lowstate, lowstate_age, lowstate_count = cache.snapshot()
            metrics = extract_motion_metrics(
                lowstate,
                lowstate_age,
                args.lowstate_fresh_s,
            )

            gate_instant = stop_gate_instant(metrics, args)
            if gate_instant:
                if stop_quiet_since is None:
                    stop_quiet_since = now
            else:
                stop_quiet_since = None

            gate_elapsed = (
                now - stop_quiet_since
                if stop_quiet_since is not None
                else 0.0
            )
            gate_ready = (
                stop_quiet_since is not None
                and gate_elapsed >= args.stop_hold_s
            )

            tele_data = tv_wrapper.get_tele_data()
            xr_valid, xr_reason = tracking_ready(
                tv_wrapper,
                tele_data,
            )

            # A normal q/Ctrl+C requests the same controlled handback as c.
            if QUIT_REQUESTED:
                QUIT_REQUESTED = False
                shutdown_after_release = True

                if state == State.LOCOMOTION_READY:
                    LOG.info(
                        "Already at ownership zero. Exiting."
                    )
                    graceful_release_completed = True
                    break

                if state in (
                    State.WAITING_FOR_STOP,
                    State.XR_ALIGNMENT,
                ):
                    state = State.LOCOMOTION_READY
                    arm_ctrl.set_weight(0.0)
                    LOG.info(
                        "Pending XR request cancelled. Exiting at weight zero."
                    )
                    graceful_release_completed = True
                    break

                if state == State.ARM_RAMP_UP:
                    begin_return(state_data, arm_ctrl, now)
                    state = State.RETURN_ARMS_HOME
                    LOG.info(
                        "Exit requested during ramp-up. Returning arms."
                    )
                elif state == State.XR_ACTIVE:
                    begin_return(state_data, arm_ctrl, now)
                    state = State.RETURN_ARMS_HOME
                    LOG.info(
                        "Exit requested. XR frozen; returning arms."
                    )
                elif state == State.SAFETY_FAULT_HOLD:
                    if gate_ready:
                        begin_return(state_data, arm_ctrl, now)
                        state = State.RETURN_ARMS_HOME
                        LOG.info(
                            "Exit requested after stop confirmation. "
                            "Returning arms."
                        )
                    else:
                        LOG.warning(
                            "Exit pending: stop the robot and keep R3 neutral."
                        )

            if TOGGLE_REQUESTED:
                TOGGLE_REQUESTED = False

                if state == State.LOCOMOTION_READY:
                    state = State.WAITING_FOR_STOP
                    state_started = now
                    LOG.info(
                        "STATE WAITING_FOR_STOP. R3 neutral; wait for the "
                        "full-stop gate."
                    )
                elif state in (
                    State.WAITING_FOR_STOP,
                    State.XR_ALIGNMENT,
                ):
                    state = State.LOCOMOTION_READY
                    state_started = now
                    align_stable_count = 0
                    align_bias_sum[:] = 0.0
                    align_bias_samples = 0
                    arm_ctrl.set_weight(0.0)
                    LOG.info(
                        "XR request cancelled. STATE LOCOMOTION_READY."
                    )
                elif state == State.ARM_RAMP_UP:
                    begin_return(state_data, arm_ctrl, now)
                    state = State.RETURN_ARMS_HOME
                    LOG.info(
                        "Ramp-up cancelled. Returning arms before release."
                    )
                elif state == State.XR_ACTIVE:
                    begin_return(state_data, arm_ctrl, now)
                    state = State.RETURN_ARMS_HOME
                    LOG.info(
                        "HANDOVER REQUESTED. XR frozen. "
                        "Returning arms to q=0."
                    )
                elif state == State.SAFETY_FAULT_HOLD:
                    if gate_ready:
                        begin_return(state_data, arm_ctrl, now)
                        state = State.RETURN_ARMS_HOME
                        LOG.info(
                            "Robot stationary. Returning arms after fault: %s.",
                            safety_fault_reason,
                        )
                    else:
                        LOG.warning(
                            "Still moving/unsettled. Handback has not started."
                        )
                else:
                    LOG.warning(
                        "A handover transition is already in progress."
                    )

            if (
                state == State.SAFETY_FAULT_HOLD
                and shutdown_after_release
                and gate_ready
            ):
                begin_return(state_data, arm_ctrl, now)
                state = State.RETURN_ARMS_HOME
                LOG.info(
                    "Pending controlled exit: full stop confirmed. "
                    "Returning arms after fault."
                )

            if state == State.WAITING_FOR_STOP and gate_ready:
                state = State.XR_ALIGNMENT
                state_started = now
                align_stable_count = 0
                align_bias_sum[:] = 0.0
                align_bias_samples = 0
                LOG.info(
                    "FULL STOP CONFIRMED. STATE XR_ALIGNMENT. "
                    "Match both hands to the robot arm pose."
                )

            if state == State.XR_ALIGNMENT:
                if (
                    lowstate is None
                    or not metrics.valid
                    or not gate_ready
                ):
                    align_stable_count = 0
                    align_bias_sum[:] = 0.0
                    align_bias_samples = 0

                    if not gate_ready:
                        state = State.WAITING_FOR_STOP
                        state_started = now
                        LOG.warning(
                            "Stop gate lost during alignment; returning to "
                            "WAITING_FOR_STOP."
                        )
                elif not xr_valid:
                    align_stable_count = 0
                    align_bias_sum[:] = 0.0
                    align_bias_samples = 0

                    if xr_reason != last_tracking_reason:
                        LOG.warning(
                            "XR alignment unavailable: %s.",
                            xr_reason,
                        )
                        last_tracking_reason = xr_reason
                else:
                    last_tracking_reason = None
                    current_q, _ = current_arm_q_dq(lowstate)
                    robot_left, robot_right = (
                        current_wrist_poses_from_fk(
                            arm_ik,
                            current_q,
                        )
                    )
                    live_left = np.asarray(
                        tele_data.left_wrist_pose,
                        dtype=float,
                    ).copy()
                    live_right = np.asarray(
                        tele_data.right_wrist_pose,
                        dtype=float,
                    ).copy()

                    candidate_bias = compute_shared_translation_bias(
                        robot_left,
                        robot_right,
                        live_left,
                        live_right,
                        "xyz",
                    )
                    registered_left = apply_translation_bias(
                        live_left,
                        candidate_bias,
                    )
                    registered_right = apply_translation_bias(
                        live_right,
                        candidate_bias,
                    )

                    left_error = float(
                        np.linalg.norm(
                            registered_left[:3, 3]
                            - robot_left[:3, 3]
                        )
                    )
                    right_error = float(
                        np.linalg.norm(
                            registered_right[:3, 3]
                            - robot_right[:3, 3]
                        )
                    )
                    bias_magnitude = float(
                        np.linalg.norm(candidate_bias)
                    )

                    alignment_ok = (
                        max(left_error, right_error)
                        <= args.start_align_m
                        and bias_magnitude
                        <= args.max_registration_shift_m
                    )

                    if alignment_ok:
                        align_stable_count += 1
                        align_bias_sum += candidate_bias
                        align_bias_samples += 1
                    else:
                        align_stable_count = 0
                        align_bias_sum[:] = 0.0
                        align_bias_samples = 0

                    if (
                        align_stable_count
                        >= args.stable_align_frames
                        and align_bias_samples > 0
                    ):
                        accepted_bias = (
                            align_bias_sum
                            / float(align_bias_samples)
                        )
                        final_left = apply_translation_bias(
                            live_left,
                            accepted_bias,
                        )
                        final_right = apply_translation_bias(
                            live_right,
                            accepted_bias,
                        )
                        xr_state.reset(
                            final_left,
                            final_right,
                            robot_left,
                            robot_right,
                        )

                        # Seed the joint target from the measured arm state.
                        arm_ctrl.set_target(
                            current_q,
                            np.zeros(14, dtype=float),
                        )
                        arm_ctrl.set_weight(0.0)

                        state = State.ARM_RAMP_UP
                        state_started = now
                        remote_fault_count = 0
                        body_fault_count = 0
                        tracking_fault_count = 0
                        lowstate_fault_count = 0

                        LOG.info(
                            "XR ALIGNMENT VALID. registration=%s.",
                            format_vector(accepted_bias),
                        )
                        LOG.warning(
                            "STATE ARM_RAMP_UP: real ownership 0 -> %.2f "
                            "over %.1f s.",
                            args.teleop_weight,
                            args.arm_ramp_up_s,
                        )

            active_control_state = state in (
                State.ARM_RAMP_UP,
                State.XR_ACTIVE,
            )

            if active_control_state:
                if metrics.valid and metrics.remote is not None:
                    remote_fault = (
                        metrics.remote.max_abs_axis
                        > args.active_r3_fault
                    )
                    body_fault = (
                        metrics.leg_waist_dq_rms_rps
                        > args.active_dq_rms_fault_rps
                        or metrics.leg_waist_dq_max_rps
                        > args.active_dq_max_fault_rps
                        or abs(metrics.gyro_z_rps)
                        > args.active_yaw_fault_rps
                    )
                    lowstate_fault_count = 0
                else:
                    remote_fault = False
                    body_fault = False
                    lowstate_fault_count += 1

                remote_fault_count = (
                    remote_fault_count + 1 if remote_fault else 0
                )
                body_fault_count = (
                    body_fault_count + 1 if body_fault else 0
                )
                tracking_fault_count = (
                    0 if xr_valid else tracking_fault_count + 1
                )

                fault_reason: Optional[str] = None
                if (
                    remote_fault_count
                    >= args.active_r3_fault_frames
                ):
                    fault_reason = "R3 joystick input during XR mode"
                elif (
                    body_fault_count
                    >= args.active_body_fault_frames
                ):
                    fault_reason = (
                        "sustained lower-body motion during XR mode"
                    )
                elif (
                    lowstate_fault_count
                    >= args.lowstate_fault_frames
                ):
                    fault_reason = "fresh LowState was lost"
                elif (
                    tracking_fault_count
                    >= args.tracking_fault_frames
                ):
                    fault_reason = f"invalid XR frame: {xr_reason}"

                if fault_reason is None and xr_valid and lowstate is not None:
                    current_q, current_dq = current_arm_q_dq(
                        lowstate
                    )
                    command_ok, command_reason = update_xr_command(
                        arm_ik=arm_ik,
                        arm_ctrl=arm_ctrl,
                        xr_state=xr_state,
                        tele_data=tele_data,
                        translation_bias=accepted_bias,
                        current_q=current_q,
                        current_dq=current_dq,
                        args=args,
                    )

                    if not command_ok:
                        fault_reason = command_reason

                if fault_reason is not None:
                    safety_fault_reason = fault_reason
                    state = State.SAFETY_FAULT_HOLD
                    state_started = now
                    LOG.error(
                        "SAFETY FAULT: %s. The last arm target and current "
                        "ownership weight are frozen. Stop the robot; when "
                        "the full-stop gate is READY, press [c].",
                        safety_fault_reason,
                    )
                elif state == State.ARM_RAMP_UP:
                    progress = (
                        now - state_started
                    ) / args.arm_ramp_up_s
                    arm_ctrl.set_weight(
                        args.teleop_weight
                        * smoothstep(progress)
                    )

                    if progress >= 1.0:
                        arm_ctrl.set_weight(args.teleop_weight)
                        state = State.XR_ACTIVE
                        state_started = now
                        LOG.warning(
                            "STATE XR_ACTIVE. Real arm ownership=%.2f. "
                            "Keep the R3 sticks neutral.",
                            args.teleop_weight,
                        )

            if xr_state.guard_active != last_guard_state:
                if xr_state.guard_active:
                    LOG.warning(
                        "Tracking jump held at %.2f m threshold.",
                        args.tracking_jump_m,
                    )
                else:
                    LOG.info("Tracking jump guard cleared.")
                last_guard_state = xr_state.guard_active

            if state == State.RETURN_ARMS_HOME:
                # The R3 operator must not move during handback. We check
                # remote input, but do not classify balance corrections caused
                # by the arm return itself as locomotion.
                remote_moved = (
                    metrics.valid
                    and metrics.remote is not None
                    and metrics.remote.max_abs_axis
                    > args.active_r3_fault
                )

                if remote_moved:
                    safety_fault_reason = (
                        "R3 input during controlled arm return"
                    )
                    state = State.SAFETY_FAULT_HOLD
                    state_started = now
                    LOG.error(
                        "HANDOVER PAUSED: %s. Last target and weight frozen.",
                        safety_fault_reason,
                    )
                else:
                    return_start = float(
                        state_data["state_started"]
                    )
                    return_start_q = np.asarray(
                        state_data["return_start_q"],
                        dtype=float,
                    )
                    elapsed = now - return_start
                    progress = elapsed / args.return_arms_s

                    target_q = return_start_q * (
                        1.0 - smoothstep(progress)
                    )
                    arm_ctrl.set_target(
                        target_q,
                        np.zeros(14, dtype=float),
                    )

                    reached = False
                    actual_error = math.inf
                    if lowstate is not None:
                        actual_q, _ = current_arm_q_dq(lowstate)
                        actual_error = float(
                            np.max(np.abs(actual_q))
                        )
                        reached = (
                            actual_error
                            <= args.return_tolerance_rad
                        )

                    timed_out = (
                        elapsed
                        >= args.return_arms_s
                        + args.return_settle_timeout_s
                    )

                    if elapsed >= args.return_arms_s and (
                        reached or timed_out
                    ):
                        if timed_out and not reached:
                            LOG.warning(
                                "Arm return tolerance was not reached; "
                                "max |q|=%.3f rad. Continuing with a slow "
                                "weight ramp-down.",
                                actual_error,
                            )
                        else:
                            LOG.info(
                                "Arms reached the handover pose; "
                                "max |q|=%.3f rad.",
                                actual_error,
                            )

                        state = State.ARM_RAMP_DOWN
                        state_started = now
                        state_data["state_started"] = now
                        state_data["return_start_weight"] = (
                            arm_ctrl.get_weight()
                        )
                        LOG.warning(
                            "STATE ARM_RAMP_DOWN: ownership %.2f -> 0 "
                            "over %.1f s.",
                            arm_ctrl.get_weight(),
                            args.arm_ramp_down_s,
                        )

            if state == State.ARM_RAMP_DOWN:
                remote_moved = (
                    metrics.valid
                    and metrics.remote is not None
                    and metrics.remote.max_abs_axis
                    > args.active_r3_fault
                )

                if remote_moved:
                    safety_fault_reason = (
                        "R3 input during arm ownership ramp-down"
                    )
                    state = State.SAFETY_FAULT_HOLD
                    state_started = now
                    LOG.error(
                        "RAMP-DOWN PAUSED: %s.",
                        safety_fault_reason,
                    )
                else:
                    start = float(state_data["state_started"])
                    start_weight = float(
                        state_data["return_start_weight"]
                    )
                    progress = (
                        now - start
                    ) / args.arm_ramp_down_s

                    arm_ctrl.set_target(
                        np.zeros(14, dtype=float),
                        np.zeros(14, dtype=float),
                    )
                    arm_ctrl.set_weight(
                        start_weight
                        * (1.0 - smoothstep(progress))
                    )

                    if progress >= 1.0:
                        arm_ctrl.set_weight(0.0)
                        state = State.LOCOMOTION_READY
                        state_started = now
                        graceful_release_completed = True

                        LOG.warning(
                            "STATE LOCOMOTION_READY. Arm ownership=0. "
                            "The R3 operator may resume moving."
                        )

                        if shutdown_after_release:
                            LOG.info(
                                "Controlled handback complete. Exiting."
                            )
                            break

            if SNAPSHOT_REQUESTED:
                SNAPSHOT_REQUESTED = False
                head_yaw_text = "INVALID"

                if pose_is_valid(
                    getattr(tele_data, "head_pose", None)
                ):
                    head_yaw_text = (
                        f"{robot_yaw_deg(np.asarray(tele_data.head_pose, dtype=float)):+.1f}"
                    )

                current_q_text = "unavailable"
                target_error_text = "unavailable"
                published_error_text = "unavailable"
                remaining_command_text = "unavailable"

                if lowstate is not None:
                    measured_q, _ = current_arm_q_dq(lowstate)
                    user_target_q = arm_ctrl.get_target_q()
                    published_user_q = arm_ctrl.get_last_sent_q()

                    current_q_text = np.array2string(
                        measured_q,
                        precision=2,
                        suppress_small=True,
                    )
                    target_error_text = (
                        f"{np.max(np.abs(user_target_q - measured_q)):.3f} rad"
                    )
                    published_error_text = (
                        f"{np.max(np.abs(published_user_q - measured_q)):.3f} rad"
                    )
                    remaining_command_text = (
                        f"{np.max(np.abs(user_target_q - published_user_q)):.3f} rad"
                    )

                LOG.info(
                    "SNAPSHOT state=%s weight=%.3f writes=%d "
                    "lowstate_count=%d age=%.4f s gate=%s %.2f/%.2f s "
                    "XR=%s head_yaw=%s deg registration=%s "
                    "IK-target-vs-measured=%s "
                    "published-user-vs-measured=%s "
                    "IK-target-vs-published=%s current_q=%s",
                    state.name,
                    arm_ctrl.get_weight(),
                    arm_ctrl.write_count,
                    lowstate_count,
                    metrics.lowstate_age_s,
                    (
                        "READY"
                        if gate_ready
                        else ("timing" if gate_instant else "moving")
                    ),
                    gate_elapsed,
                    args.stop_hold_s,
                    xr_valid,
                    head_yaw_text,
                    format_vector(accepted_bias),
                    target_error_text,
                    published_error_text,
                    remaining_command_text,
                    current_q_text,
                )

            if now - last_status >= 1.0 / args.status_hz:
                remote = metrics.remote or RemoteState()
                LOG.info(
                    "state=%-18s | weight=%.3f | gate=%-6s %.2f/%.2f | "
                    "dq max/rms=%s/%s | yaw=%s | "
                    "R3=%s/%s/%s/%s max=%s | XR=%s | guard=%s",
                    state.name,
                    arm_ctrl.get_weight(),
                    (
                        "READY"
                        if gate_ready
                        else ("timing" if gate_instant else "moving")
                    ),
                    gate_elapsed,
                    args.stop_hold_s,
                    (
                        f"{metrics.leg_waist_dq_max_rps:.3f}"
                        if math.isfinite(
                            metrics.leg_waist_dq_max_rps
                        )
                        else "?"
                    ),
                    (
                        f"{metrics.leg_waist_dq_rms_rps:.3f}"
                        if math.isfinite(
                            metrics.leg_waist_dq_rms_rps
                        )
                        else "?"
                    ),
                    (
                        f"{metrics.gyro_z_rps:+.3f}"
                        if math.isfinite(metrics.gyro_z_rps)
                        else "?"
                    ),
                    f"{remote.lx:+.2f}" if remote.valid else "?",
                    f"{remote.ly:+.2f}" if remote.valid else "?",
                    f"{remote.rx:+.2f}" if remote.valid else "?",
                    f"{remote.ry:+.2f}" if remote.valid else "?",
                    (
                        f"{remote.max_abs_axis:.2f}"
                        if remote.valid
                        else "?"
                    ),
                    "ok" if xr_valid else xr_reason,
                    "HOLD" if xr_state.guard_active else "ok",
                )
                last_status = now

            if arm_ctrl.last_write_exception is not None:
                raise RuntimeError(
                    "rt/arm_sdk publisher failed"
                ) from arm_ctrl.last_write_exception

            sleep_time = max(
                0.0,
                (1.0 / args.frequency)
                - (time.monotonic() - loop_start),
            )
            FORCE_STOP.wait(sleep_time)

        return 0

    except KeyboardInterrupt:
        LOG.warning("KeyboardInterrupt.")
        return 1
    except Exception:
        LOG.exception("Fatal error")
        return 1
    finally:
        try:
            stop_listening()
        except Exception:
            pass

        if (
            arm_ctrl is not None
            and arm_ctrl.get_weight() > 1e-4
            and not graceful_release_completed
        ):
            LOG.error(
                "Abnormal exit with arm weight %.3f. "
                "Ramping weight to zero without changing the frozen target.",
                arm_ctrl.get_weight(),
            )
            try:
                arm_ctrl.release_weight_only(duration_s=1.0)
            except Exception as exc:
                LOG.error("Emergency weight release failed: %s", exc)

        if arm_ctrl is not None:
            try:
                arm_ctrl.close()
            except Exception as exc:
                LOG.error("Arm controller close failed: %s", exc)

        if keyboard_thread is not None and keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)

        if lowstate_subscriber is not None:
            try:
                lowstate_subscriber.Close()
            except Exception as exc:
                LOG.warning("LowState subscriber close failed: %s", exc)

        if tv_wrapper is not None:
            try:
                tv_wrapper.close()
            except Exception as exc:
                LOG.warning("TeleVuer close failed: %s", exc)

        LOG.info("Exited.")


if __name__ == "__main__":
    raise SystemExit(main())
