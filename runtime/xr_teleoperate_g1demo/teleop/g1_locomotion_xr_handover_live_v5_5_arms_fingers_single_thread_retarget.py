#!/usr/bin/env python3
"""Live G1 Regular-mode locomotion <-> stationary XR arm handover.

THIS PROGRAM PUBLISHES REAL ARM COMMANDS TO ``rt/arm_sdk`` AND REAL
INSPIRE DFX COMMANDS TO ``rt/inspire/cmd``.

Merged from verified sources:
- arm V4 SHA-256: 915118ee63dffbd0e820aeb116105a70e0fac36bf72a5755d53acb6335f49b67
- finger V5 SHA-256: dbc9c43dcc1ddb98bbc9ad8c508140cd519b4ab3b76e9517a6cb68f70ee0260e

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
    The XR operator controls both arms and both Inspire DFX hands. Fingers run
    in a dedicated 60 Hz worker while the arm safety loop remains separate.

RETURN_ARMS_HOME
    XR is frozen and the commanded 14-joint arm target is moved smoothly to
    the Regular-mode handover pose q=0.

ARM_RAMP_DOWN
    Ownership is ramped to zero, returning the arms to the stock controller.

Safety model
------------
- R3 movement during XR control freezes the XR command and enters a fault hold.
- Sustained lower-body movement during XR control also enters a fault hold.
- Invalid XR arm frames freeze the arm command and enter a fault hold.
- Persistent finger-tracking loss smoothly opens both hands.
- Stale Inspire feedback freezes finger commands and faults active XR control.
- The left thumb-rotation command is permanently locked open at 1.0.
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
    https://192.168.0.116:8012/?ws=wss://192.168.0.116:8012
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import socket
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
from unitree_sdk2py.idl.default import (
    unitree_go_msg_dds__MotorCmd_,
    unitree_hg_msg_dds__LowCmd_,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
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
INSPIRE_MOTOR_COUNT = 12
INSPIRE_COMMAND_TOPIC = "rt/inspire/cmd"
INSPIRE_STATE_TOPIC = "rt/inspire/state"
INSPIRE_RIGHT_IDS = tuple(range(0, 6))
INSPIRE_LEFT_IDS = tuple(range(6, 12))


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_locomotion_xr_arms_fingers_v5")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)-8s [ArmsFingersV5] %(message)s",
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


class FingerMode(Enum):
    OPEN = auto()
    FOLLOW = auto()
    HOLD = auto()


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
    initial_ik_seed_pending: bool = False

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
        self.initial_ik_seed_pending = True


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



def apply_bimanual_inward_offset(
    left_pose: np.ndarray,
    right_pose: np.ndarray,
    per_wrist_offset_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Move both wrist targets inward while preserving their midpoint."""
    left = np.asarray(left_pose, dtype=float).copy()
    right = np.asarray(right_pose, dtype=float).copy()

    requested = float(max(0.0, per_wrist_offset_m))
    if requested <= 0.0:
        return left, right

    delta = right[:3, 3] - left[:3, 3]
    separation = float(np.linalg.norm(delta))
    if not math.isfinite(separation) or separation <= 1e-6:
        return left, right

    # Never allow the correction to cross the wrist centers. The CLI also
    # limits the requested correction to 5 cm per wrist.
    applied = min(requested, 0.45 * separation)
    direction = delta / separation

    left[:3, 3] += direction * applied
    right[:3, 3] -= direction * applied
    return left, right


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

    registered_left, registered_right = apply_bimanual_inward_offset(
        registered_left,
        registered_right,
        args.bimanual_inward_offset_m,
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

    if xr_state.initial_ik_seed_pending:
        # The first IK solution after alignment initializes the user target;
        # it is not a branch jump. Ownership is still zero at this point.
        initial_ik_offset = float(
            np.max(np.abs(solution_q - current_q))
        )
        if initial_ik_offset > args.max_initial_ik_offset_rad:
            return (
                False,
                "initial IK offset "
                f"{initial_ik_offset:.3f} rad exceeds "
                f"{args.max_initial_ik_offset_rad:.3f} rad",
            )

        arm_ctrl.set_target(solution_q, solution_tau)
        xr_state.initial_ik_seed_pending = False
        LOG.info(
            "Initial IK target seeded at weight zero; "
            "max joint offset=%.3f rad.",
            initial_ik_offset,
        )
        return True, "ok"

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



def finite_hand_landmarks(
    value: object,
) -> tuple[bool, Optional[np.ndarray], str]:
    try:
        data = np.asarray(value, dtype=float)
    except Exception as exc:
        return False, None, f"conversion failed: {exc}"

    if data.shape != (25, 3):
        return False, None, f"shape={data.shape}, expected (25, 3)"
    if not np.isfinite(data).all():
        return False, None, "contains NaN/Inf"

    extents = np.ptp(data, axis=0)
    max_extent = float(np.max(extents))
    if max_extent < 0.025:
        return False, None, f"collapsed landmarks, extent={max_extent:.4f} m"

    return True, data.copy(), "ok"


def normalize_inspire_target(raw_target: np.ndarray) -> np.ndarray:
    raw = np.asarray(raw_target, dtype=float).copy()
    if raw.shape != (6,):
        raise ValueError(f"Expected six Inspire targets, got {raw.shape}")

    result = np.empty(6, dtype=float)
    result[:4] = (1.7 - raw[:4]) / 1.7
    result[4] = (0.5 - raw[4]) / 0.5
    result[5] = (1.3 - raw[5]) / 1.4
    return np.clip(result, 0.0, 1.0)


def rate_limit_finger(
    current: np.ndarray,
    target: np.ndarray,
    max_speed_per_s: float,
    dt: float,
) -> np.ndarray:
    step = max(0.0, float(max_speed_per_s)) * max(0.0, float(dt))
    return current + np.clip(target - current, -step, step)


class SharedHandTracking:
    """Thread-safe copy of the latest TeleVuer hand landmarks.

    Only the main arm loop calls TeleVuer. The 60 Hz finger worker consumes
    copies from this cache, so no second TeleVuer instance or concurrent
    get_tele_data() call is created.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._left: Optional[np.ndarray] = None
        self._right: Optional[np.ndarray] = None
        self._motion_ready = False
        self._received_monotonic = 0.0
        self._sequence = 0

    def update(self, tele_data: object) -> None:
        left_value = getattr(tele_data, "left_hand_pos", None)
        right_value = getattr(tele_data, "right_hand_pos", None)

        try:
            left = np.asarray(left_value, dtype=float).copy()
        except Exception:
            left = None
        try:
            right = np.asarray(right_value, dtype=float).copy()
        except Exception:
            right = None

        with self._lock:
            self._left = left
            self._right = right
            self._motion_ready = bool(
                getattr(tele_data, "motion_data_ready", False)
            )
            self._received_monotonic = time.monotonic()
            self._sequence += 1

    def snapshot(
        self,
    ) -> tuple[
        Optional[np.ndarray],
        Optional[np.ndarray],
        bool,
        float,
        int,
    ]:
        with self._lock:
            left = None if self._left is None else self._left.copy()
            right = None if self._right is None else self._right.copy()
            motion_ready = self._motion_ready
            received = self._received_monotonic
            sequence = self._sequence

        age = (
            time.monotonic() - received
            if received > 0.0
            else math.inf
        )
        return left, right, motion_ready, age, sequence


class InspireDfxIo:
    def __init__(self) -> None:
        self.publisher = ChannelPublisher(
            INSPIRE_COMMAND_TOPIC,
            MotorCmds_,
        )
        self.publisher.Init()

        self.subscriber = ChannelSubscriber(
            INSPIRE_STATE_TOPIC,
            MotorStates_,
        )
        self.subscriber.Init()

        self.message = MotorCmds_()
        self.message.cmds = [
            unitree_go_msg_dds__MotorCmd_()
            for _ in range(INSPIRE_MOTOR_COUNT)
        ]

        self.last_state: Optional[np.ndarray] = None
        self.last_state_time = 0.0
        self.write_count = 0
        self.last_write_exception: Optional[Exception] = None

    def publish(self, left: np.ndarray, right: np.ndarray) -> None:
        left_command = np.clip(
            np.asarray(left, dtype=float),
            0.0,
            1.0,
        ).copy()
        right_command = np.clip(
            np.asarray(right, dtype=float),
            0.0,
            1.0,
        ).copy()
        if left_command.shape != (6,) or right_command.shape != (6,):
            raise ValueError("Left/right finger commands require six values")

        # Hardware-side fault workaround: never command left thumb rotation.
        left_command[5] = 1.0

        for index, motor_id in enumerate(INSPIRE_RIGHT_IDS):
            self.message.cmds[motor_id].q = float(right_command[index])
        for index, motor_id in enumerate(INSPIRE_LEFT_IDS):
            self.message.cmds[motor_id].q = float(left_command[index])

        try:
            self.publisher.Write(self.message)
            self.write_count += 1
        except Exception as exc:
            self.last_write_exception = exc
            raise

    def poll_state(self) -> Optional[np.ndarray]:
        message = self.subscriber.Read()
        if message is None:
            return self.last_state

        try:
            values = np.asarray(
                [
                    message.states[index].q
                    for index in range(INSPIRE_MOTOR_COUNT)
                ],
                dtype=float,
            )
        except Exception as exc:
            LOG.warning("Could not decode Inspire state: %s", exc)
            return self.last_state

        if np.isfinite(values).all():
            self.last_state = values
            self.last_state_time = time.monotonic()

        return self.last_state

    def state_age_s(self) -> float:
        if self.last_state_time <= 0.0:
            return math.inf
        return time.monotonic() - self.last_state_time

    def close(self) -> None:
        try:
            self.publisher.Close()
        except Exception:
            pass
        try:
            self.subscriber.Close()
        except Exception:
            pass


def wait_for_inspire_state(io: InspireDfxIo, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if io.poll_state() is not None:
            LOG.info("Inspire state feedback received.")
            return
        time.sleep(0.01)

    raise TimeoutError(
        f"No {INSPIRE_STATE_TOPIC} feedback within {timeout_s:.1f} seconds. "
        "Confirm inspire_g1 is running with LD_LIBRARY_PATH=/usr/local/lib."
    )


class FingerController:
    """Inspire controller whose 60 Hz retargeting worker starts lazily.

    The DDS hand interface is initialized immediately and both hands are
    commanded open, but the heavy dex-retargeting import and worker thread are
    deferred until TeleVuer has delivered a valid XR frame. This keeps the
    TeleVuer/arm startup path equivalent to the proven arm V4 path.
    """

    def __init__(
        self,
        tracking: SharedHandTracking,
        *,
        frequency_hz: float,
        command_speed_per_s: float,
        minimum_command: float,
        stable_tracking_frames: int,
        tracking_fault_frames: int,
        tracking_stale_s: float,
        feedback_stale_s: float,
        feedback_timeout_s: float,
    ) -> None:
        self._tracking = tracking
        self._frequency_hz = float(frequency_hz)
        self._command_speed_per_s = float(command_speed_per_s)
        self._minimum_command = float(minimum_command)
        self._stable_tracking_frames = int(stable_tracking_frames)
        self._tracking_fault_frames = int(tracking_fault_frames)
        self._tracking_stale_s = float(tracking_stale_s)
        self._feedback_stale_s = float(feedback_stale_s)

        self._lock = threading.Lock()
        self._start_lock = threading.Lock()
        self._mode = FingerMode.OPEN
        self._stop = threading.Event()
        self._worker_error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._start_requested = False
        self._last_deferred_publish = 0.0
        self._status: dict[str, object] = {
            "mode": FingerMode.OPEN,
            "worker_started": False,
            "worker_initializing": False,
            "worker_phase": "deferred",
            "worker_heartbeat_monotonic": 0.0,
            "worker_age_s": math.inf,
            "worker_loop_count": 0,
            "retarget_count": 0,
            "tracking_valid": False,
            "tracking_reason": "finger worker deferred until valid XR frame",
            "stable_count": 0,
            "fault_count": 0,
            "feedback_age_s": math.inf,
            "feedback_fault": False,
            "current_left": np.ones(6, dtype=float),
            "current_right": np.ones(6, dtype=float),
            "target_left": np.ones(6, dtype=float),
            "target_right": np.ones(6, dtype=float),
            "raw_left": np.full(6, np.nan, dtype=float),
            "raw_right": np.full(6, np.nan, dtype=float),
            "normalized_left": np.ones(6, dtype=float),
            "normalized_right": np.ones(6, dtype=float),
            "state": None,
            "write_count": 0,
            "worker_error": None,
        }

        self._io = InspireDfxIo()
        try:
            wait_for_inspire_state(self._io, feedback_timeout_s)

            # Establish and verify a deterministic open command without
            # starting dex-retargeting or a background worker.
            for _ in range(10):
                self._io.publish(np.ones(6), np.ones(6))
                self._io.poll_state()
                time.sleep(0.02)

            self._update_status(
                feedback_age_s=self._io.state_age_s(),
                state=self._io.last_state,
                write_count=self._io.write_count,
            )
        except Exception:
            self._io.close()
            raise

    def worker_started(self) -> bool:
        with self._lock:
            return bool(self._status["worker_started"])

    def request_worker_start(self) -> None:
        """Start one thread that constructs and uses the retargeter."""
        if self.worker_started():
            return

        with self._start_lock:
            if self.worker_started() or self._start_requested:
                return
            if self._stop.is_set():
                raise RuntimeError("Cannot start a stopped finger controller")

            self._start_requested = True
            with self._lock:
                self._status["worker_initializing"] = True
                self._status["worker_phase"] = "initializing"
                self._status["tracking_reason"] = (
                    "loading Inspire retargeting in finger worker"
                )

            LOG.info(
                "Valid Quest XR frame received. Starting the dedicated "
                "finger thread; retargeting will be constructed and used "
                "inside that same thread."
            )

            self._thread = threading.Thread(
                target=self._initialize_and_run,
                name="g1-inspire-finger-worker",
                daemon=True,
            )
            self._thread.start()

    def _initialize_and_run(self) -> None:
        """Construct retargeting and execute its loop in this same thread."""
        try:
            from teleop.robot_control.hand_retargeting import (
                HandRetargeting,
                HandType,
            )

            retargeting = HandRetargeting(HandType.INSPIRE_HAND)

            if self._stop.is_set():
                self._update_status(
                    worker_initializing=False,
                    worker_phase="stopped-before-run",
                    tracking_reason=(
                        "retargeting initialization cancelled during shutdown"
                    ),
                )
                return

            now = time.monotonic()
            with self._lock:
                self._status["worker_started"] = True
                self._status["worker_initializing"] = False
                self._status["worker_phase"] = "starting"
                self._status["worker_heartbeat_monotonic"] = now
                self._status["worker_age_s"] = 0.0
                self._status["tracking_reason"] = "worker starting"

            LOG.info(
                "Inspire retargeting initialized in the finger thread. "
                "The same thread is now running the 60 Hz finger loop."
            )
            self._run(retargeting)
        except Exception as exc:
            self._worker_error = (
                f"finger-thread start {type(exc).__name__}: {exc}"
            )
            self._update_status(
                worker_initializing=False,
                worker_phase="failed",
                tracking_reason=self._worker_error,
            )
            LOG.exception("Finger thread initialization/run failed")

    def service_deferred(self) -> None:
        """Keep hands open and feedback fresh before worker startup."""
        if self.worker_started() or self._stop.is_set():
            return

        now = time.monotonic()
        state = self._io.poll_state()
        if now - self._last_deferred_publish >= 0.10:
            self._io.publish(np.ones(6), np.ones(6))
            self._last_deferred_publish = now

        feedback_age = self._io.state_age_s()
        with self._lock:
            initializing = bool(self._status["worker_initializing"])
        deferred_reason = (
            "loading Inspire retargeting in finger worker"
            if initializing
            else "finger worker deferred until valid XR frame"
        )
        self._update_status(
            tracking_reason=deferred_reason,
            feedback_age_s=feedback_age,
            feedback_fault=feedback_age > self._feedback_stale_s,
            current_left=np.ones(6, dtype=float),
            current_right=np.ones(6, dtype=float),
            target_left=np.ones(6, dtype=float),
            target_right=np.ones(6, dtype=float),
            state=state,
            write_count=self._io.write_count,
        )

    def set_mode(self, mode: FingerMode) -> None:
        with self._lock:
            self._mode = mode

    def get_mode(self) -> FingerMode:
        with self._lock:
            return self._mode

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            result: dict[str, object] = {}
            for key, value in self._status.items():
                result[key] = (
                    value.copy() if isinstance(value, np.ndarray) else value
                )
            result["mode"] = self._mode

        heartbeat = float(result["worker_heartbeat_monotonic"])
        result["worker_age_s"] = (
            time.monotonic() - heartbeat
            if heartbeat > 0.0
            else math.inf
        )
        return result

    def alignment_ready(self) -> tuple[bool, str]:
        status = self.snapshot()
        if bool(status["worker_initializing"]):
            return False, "finger retargeting is still initializing"
        if not bool(status["worker_started"]):
            return False, "finger worker has not started"
        if status["worker_error"] is not None:
            return False, f"finger worker error: {status['worker_error']}"
        if bool(status["feedback_fault"]):
            return False, (
                "Inspire feedback stale: "
                f"{float(status['feedback_age_s']):.3f} s"
            )
        if not bool(status["tracking_valid"]):
            return False, str(status["tracking_reason"])
        stable_count = int(status["stable_count"])
        if stable_count < self._stable_tracking_frames:
            return False, "finger tracking stabilizing"
        return True, "ok"

    def request_open_and_wait(self, timeout_s: float) -> bool:
        self.set_mode(FingerMode.OPEN)

        # Before worker startup, publish open directly and synchronously.
        thread = self._thread
        if thread is None:
            deadline = time.monotonic() + max(0.0, timeout_s)
            while time.monotonic() < deadline:
                self._io.publish(np.ones(6), np.ones(6))
                state = self._io.poll_state()
                self._update_status(
                    current_left=np.ones(6, dtype=float),
                    current_right=np.ones(6, dtype=float),
                    target_left=np.ones(6, dtype=float),
                    target_right=np.ones(6, dtype=float),
                    feedback_age_s=self._io.state_age_s(),
                    state=state,
                    write_count=self._io.write_count,
                )
                return True
            return False

        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            status = self.snapshot()
            left = np.asarray(status["current_left"], dtype=float)
            right = np.asarray(status["current_right"], dtype=float)
            if (
                float(np.min(left)) >= 0.995
                and float(np.min(right)) >= 0.995
            ):
                return True
            if not thread.is_alive():
                break
            time.sleep(0.02)
        return False

    def close(self) -> None:
        self._stop.set()

        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.5)
            if thread.is_alive():
                LOG.error(
                    "Finger worker did not stop within 1.5 seconds."
                )

        # Final repeated open samples are independent of worker state.
        for _ in range(10):
            try:
                self._io.publish(np.ones(6), np.ones(6))
            except Exception:
                break
            time.sleep(0.02)

        self._io.close()

    def _update_status(self, **values: object) -> None:
        with self._lock:
            self._status.update(values)
            self._status["worker_error"] = self._worker_error

    def _run(self, retargeting: object) -> None:
        current_left = np.ones(6, dtype=float)
        current_right = np.ones(6, dtype=float)
        target_left = np.ones(6, dtype=float)
        target_right = np.ones(6, dtype=float)
        last_raw_left = np.full(6, np.nan, dtype=float)
        last_raw_right = np.full(6, np.nan, dtype=float)
        last_normalized_left = np.ones(6, dtype=float)
        last_normalized_right = np.ones(6, dtype=float)

        stable_count = 0
        fault_count = 0
        last_sequence = -1
        last_loop = time.monotonic()
        tracking_fault_latched = False
        feedback_fault_latched = False
        reopen_after_feedback = False
        worker_loop_count = 0
        retarget_count = 0

        try:
            while not self._stop.is_set():
                loop_start = time.monotonic()
                worker_loop_count += 1
                self._update_status(
                    worker_phase="loop-start",
                    worker_heartbeat_monotonic=loop_start,
                    worker_loop_count=worker_loop_count,
                    retarget_count=retarget_count,
                )
                dt = min(0.1, max(0.0, loop_start - last_loop))
                last_loop = loop_start

                mode = self.get_mode()
                state_values = self._io.poll_state()
                feedback_age = self._io.state_age_s()
                feedback_fault = feedback_age > self._feedback_stale_s

                if feedback_fault and not feedback_fault_latched:
                    LOG.error(
                        "INSPIRE FEEDBACK STALE: age=%.3f s. Finger commands "
                        "are frozen until feedback returns.",
                        feedback_age,
                    )
                    feedback_fault_latched = True
                    reopen_after_feedback = True
                elif not feedback_fault and feedback_fault_latched:
                    LOG.warning(
                        "Inspire feedback recovered. Opening both hands before "
                        "finger following can resume."
                    )
                    feedback_fault_latched = False

                (
                    left_value,
                    right_value,
                    motion_ready,
                    tracking_age,
                    sequence,
                ) = self._tracking.snapshot()

                left_valid, left_hand, left_reason = finite_hand_landmarks(
                    left_value
                )
                right_valid, right_hand, right_reason = finite_hand_landmarks(
                    right_value
                )
                tracking_valid = (
                    motion_ready
                    and tracking_age <= self._tracking_stale_s
                    and left_valid
                    and right_valid
                    and left_hand is not None
                    and right_hand is not None
                )
                tracking_reason = (
                    "ok"
                    if tracking_valid
                    else (
                        f"motion_ready={motion_ready}; "
                        f"age={tracking_age:.3f}s; "
                        f"left={left_reason}; right={right_reason}"
                    )
                )

                if sequence != last_sequence:
                    last_sequence = sequence
                    if tracking_valid:
                        stable_count += 1
                        fault_count = 0
                        tracking_fault_latched = False
                    else:
                        stable_count = 0
                        fault_count += 1

                if (
                    mode == FingerMode.FOLLOW
                    and fault_count >= self._tracking_fault_frames
                    and not tracking_fault_latched
                ):
                    LOG.error(
                        "FINGER TRACKING FAULT. Opening both hands. %s",
                        tracking_reason,
                    )
                    tracking_fault_latched = True

                effective_mode = mode
                if feedback_fault:
                    effective_mode = FingerMode.HOLD
                elif reopen_after_feedback:
                    effective_mode = FingerMode.OPEN
                elif (
                    mode == FingerMode.FOLLOW
                    and fault_count >= self._tracking_fault_frames
                ):
                    effective_mode = FingerMode.OPEN

                if effective_mode == FingerMode.FOLLOW and tracking_valid:
                    assert left_hand is not None
                    assert right_hand is not None

                    ref_left = (
                        left_hand[retargeting.left_indices[1, :]]
                        - left_hand[retargeting.left_indices[0, :]]
                    )
                    ref_right = (
                        right_hand[retargeting.right_indices[1, :]]
                        - right_hand[retargeting.right_indices[0, :]]
                    )

                    self._update_status(
                        worker_phase="retarget-left",
                        worker_heartbeat_monotonic=time.monotonic(),
                    )
                    raw_left = (
                        retargeting.left_retargeting.retarget(ref_left)[
                            retargeting.left_dex_retargeting_to_hardware
                        ]
                    )

                    self._update_status(
                        worker_phase="retarget-right",
                        worker_heartbeat_monotonic=time.monotonic(),
                    )
                    raw_right = (
                        retargeting.right_retargeting.retarget(ref_right)[
                            retargeting.right_dex_retargeting_to_hardware
                        ]
                    )
                    retarget_count += 1

                    last_raw_left = np.asarray(
                        raw_left,
                        dtype=float,
                    ).copy()
                    last_raw_right = np.asarray(
                        raw_right,
                        dtype=float,
                    ).copy()

                    target_left = normalize_inspire_target(raw_left)
                    target_right = normalize_inspire_target(raw_right)
                    last_normalized_left = target_left.copy()
                    last_normalized_right = target_right.copy()

                    target_left = np.maximum(
                        target_left,
                        self._minimum_command,
                    )
                    target_right = np.maximum(
                        target_right,
                        self._minimum_command,
                    )

                    # Hardware-side fault workaround, before rate limiting.
                    target_left[5] = 1.0
                elif effective_mode == FingerMode.OPEN:
                    target_left = np.ones(6, dtype=float)
                    target_right = np.ones(6, dtype=float)
                else:
                    target_left = current_left.copy()
                    target_right = current_right.copy()

                if not feedback_fault:
                    current_left = rate_limit_finger(
                        current_left,
                        target_left,
                        self._command_speed_per_s,
                        dt,
                    )
                    current_right = rate_limit_finger(
                        current_right,
                        target_right,
                        self._command_speed_per_s,
                        dt,
                    )
                    current_left[5] = 1.0

                    self._update_status(
                        worker_phase="publish",
                        worker_heartbeat_monotonic=time.monotonic(),
                    )
                    self._io.publish(current_left, current_right)

                    if (
                        reopen_after_feedback
                        and float(np.min(current_left)) >= 0.995
                        and float(np.min(current_right)) >= 0.995
                    ):
                        reopen_after_feedback = False

                self._update_status(
                    mode=mode,
                    worker_started=True,
                    worker_phase="sleep",
                    worker_heartbeat_monotonic=time.monotonic(),
                    worker_loop_count=worker_loop_count,
                    retarget_count=retarget_count,
                    tracking_valid=tracking_valid,
                    tracking_reason=tracking_reason,
                    stable_count=stable_count,
                    fault_count=fault_count,
                    feedback_age_s=feedback_age,
                    feedback_fault=feedback_fault,
                    current_left=current_left,
                    current_right=current_right,
                    target_left=target_left,
                    target_right=target_right,
                    raw_left=last_raw_left,
                    raw_right=last_raw_right,
                    normalized_left=last_normalized_left,
                    normalized_right=last_normalized_right,
                    state=state_values,
                    write_count=self._io.write_count,
                )

                elapsed = time.monotonic() - loop_start
                self._stop.wait(
                    max(0.0, 1.0 / self._frequency_hz - elapsed)
                )
        except Exception as exc:
            self._worker_error = f"{type(exc).__name__}: {exc}"
            self._update_status()
            LOG.exception("Finger worker failed")




def finger_mode_for_arm_state(state: State) -> FingerMode:
    if state == State.XR_ACTIVE:
        return FingerMode.FOLLOW
    if state in (State.SAFETY_FAULT_HOLD, State.RETURN_ARMS_HOME):
        return FingerMode.HOLD
    return FingerMode.OPEN


def direct_child_pids(pid: int) -> list[int]:
    path = Path(f"/proc/{pid}/task/{pid}/children")
    try:
        content = path.read_text().strip()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return []

    result: list[int] = []
    for token in content.split():
        try:
            child = int(token)
        except ValueError:
            continue
        if child > 1 and child != os.getpid():
            result.append(child)
    return result


def descendant_pids(root_pid: int) -> list[int]:
    ordered: list[int] = []
    seen: set[int] = set()
    stack = list(reversed(direct_child_pids(root_pid)))

    while stack:
        pid = stack.pop()
        if pid in seen:
            continue
        seen.add(pid)
        ordered.append(pid)
        stack.extend(reversed(direct_child_pids(pid)))

    return ordered


def process_label(pid: int) -> str:
    try:
        data = Path(f"/proc/{pid}/cmdline").read_bytes()
        label = data.replace(b"\0", b" ").decode(
            errors="replace"
        ).strip()
        return label or f"[pid {pid}]"
    except Exception:
        return f"[pid {pid}]"


def kill_all_descendants() -> None:
    root_pid = os.getpid()

    for attempt in range(1, 4):
        descendants = descendant_pids(root_pid)
        if not descendants:
            if attempt == 1:
                LOG.info("No remaining child processes.")
            return

        LOG.warning(
            "Shutdown pass %d: terminating %d descendant process(es): %s",
            attempt,
            len(descendants),
            ", ".join(
                f"{pid}:{process_label(pid)}"
                for pid in descendants
            ),
        )

        for pid in reversed(descendants):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError as exc:
                LOG.warning(
                    "Could not kill descendant pid=%d: %s",
                    pid,
                    exc,
                )

        time.sleep(0.15)

    survivors = descendant_pids(root_pid)
    if survivors:
        LOG.error(
            "Descendant processes still alive: %s",
            ", ".join(
                f"{pid}:{process_label(pid)}"
                for pid in survivors
            ),
        )
    else:
        LOG.info("All descendant processes stopped.")


def require_tcp_port_free(
    port: int = 8012,
    host: str = "0.0.0.0",
) -> None:
    """Fail before TeleVuer startup if another listener owns the port."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
        probe.bind((host, int(port)))
    except OSError as exc:
        raise RuntimeError(
            f"TCP port {port} is already in use. "
            "A stale TeleVuer/Vuer process is probably still running. "
            "Run: sudo ss -ltnp 'sport = :8012' ; "
            "sudo fuser -v 8012/tcp"
        ) from exc
    finally:
        probe.close()



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
            "Live G1 XR arm + Inspire finger control with controlled handback."
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
        "--max-initial-ik-offset-rad",
        type=float,
        default=0.65,
        help=(
            "Maximum measured-to-first-IK joint offset accepted at weight "
            "zero before the ownership ramp begins."
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
        "--bimanual-inward-offset-m",
        type=float,
        default=0.0,
        help=(
            "Symmetric inward correction applied to each wrist target. "
            "For example, 0.02 reduces total wrist separation by about 4 cm."
        ),
    )
    parser.add_argument(
        "--allow-locomotion-during-xr",
        action="store_true",
        help=(
            "Allow R3/lower-body locomotion while XR arm control is active. "
            "Tracking, LowState, IK, and command guards remain enabled."
        ),
    )

    parser.add_argument(
        "--active-r3-fault",
        type=float,
        default=0.12,
    )
    parser.add_argument(
        "--active-r3-fault-frames",
        type=int,
        default=1,
    )
    parser.add_argument(
        "--active-dq-rms-fault-rps",
        type=float,
        default=0.18,
    )
    parser.add_argument(
        "--active-dq-max-fault-rps",
        type=float,
        default=0.75,
    )
    parser.add_argument(
        "--active-yaw-fault-rps",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--active-body-fault-frames",
        type=int,
        default=3,
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

    parser.add_argument(
        "--finger-frequency",
        type=float,
        default=60.0,
    )
    parser.add_argument(
        "--finger-command-speed-per-s",
        type=float,
        default=1.00,
    )
    parser.add_argument(
        "--finger-minimum-command",
        type=float,
        default=0.20,
    )
    parser.add_argument(
        "--finger-stable-tracking-frames",
        type=int,
        default=15,
    )
    parser.add_argument(
        "--finger-tracking-fault-frames",
        type=int,
        default=3,
    )
    parser.add_argument(
        "--finger-tracking-stale-s",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--finger-state-stale-s",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--finger-state-timeout-s",
        type=float,
        default=5.0,
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
        "max_initial_ik_offset_rad",
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
        "finger_frequency",
        "finger_command_speed_per_s",
        "finger_tracking_stale_s",
        "finger_state_stale_s",
        "finger_state_timeout_s",
    )
    for name in positive_names:
        if getattr(args, name) <= 0:
            raise ValueError(
                f"--{name.replace('_', '-')} must be positive"
            )

    if not 0.0 < args.teleop_weight <= 1.0:
        raise ValueError("--teleop-weight must be in (0, 1]")

    if not 0.0 <= args.finger_minimum_command < 1.0:
        raise ValueError("--finger-minimum-command must be in [0, 1)")

    if not 0.0 <= args.bimanual_inward_offset_m <= 0.05:
        raise ValueError(
            "--bimanual-inward-offset-m must be between 0 and 0.05"
        )

    integer_names = (
        "stable_align_frames",
        "tracking_reacquire_frames",
        "active_r3_fault_frames",
        "active_body_fault_frames",
        "tracking_fault_frames",
        "lowstate_fault_frames",
        "finger_stable_tracking_frames",
        "finger_tracking_fault_frames",
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
    finger_ctrl: Optional[FingerController] = None
    hand_tracking = SharedHandTracking()

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

        LOG.info("G1_LOCOMOTION_XR_HANDOVER_LIVE_V5_5_ARMS_FINGERS_SINGLE_THREAD_RETARGET")
        LOG.warning(
            "LIVE ARM COMMANDS ENABLED: publishing rt/arm_sdk."
        )
        LOG.info(
            "V4 limiter active: user arm commands accumulate from the "
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
        if args.allow_locomotion_during_xr:
            LOG.warning(
                "CONCURRENT LOCOMOTION ENABLED: R3 and lower-body movement "
                "will not fault XR arm control. [c] freezes XR and starts "
                "an uninterrupted controlled arm handback; it does not stop "
                "the robot base."
            )
        else:
            LOG.info(
                "Active interlock: R3>%.2f for %d frame(s); "
                "body RMS>%.2f or max>%.2f or |yaw|>%.2f for %d frame(s).",
                args.active_r3_fault,
                args.active_r3_fault_frames,
                args.active_dq_rms_fault_rps,
                args.active_dq_max_fault_rps,
                args.active_yaw_fault_rps,
                args.active_body_fault_frames,
            )

        LOG.info(
            "Bimanual inward correction=%.3f m per wrist "
            "(approximately %.3f m total separation reduction).",
            args.bimanual_inward_offset_m,
            2.0 * args.bimanual_inward_offset_m,
        )
        LOG.warning(
            "LIVE INSPIRE COMMANDS ENABLED: publishing %s.",
            INSPIRE_COMMAND_TOPIC,
        )
        LOG.warning(
            "LEFT THUMB ROTATION DISABLED: fixed command=1.0."
        )
        LOG.info(
            "Finger loop=%.1f Hz | command speed<=%.2f/s | "
            "minimum command=%.2f | state stale fault=%.2f s",
            args.finger_frequency,
            args.finger_command_speed_per_s,
            args.finger_minimum_command,
            args.finger_state_stale_s,
        )
        LOG.info(
            "Open only: "
            "https://192.168.0.116:8012/?ws=wss://192.168.0.116:8012"
        )

        # Do not let a stale TeleVuer child silently serve the Quest while
        # this controller fails to bind. Abort clearly before startup instead.
        require_tcp_port_free(port=8012)
        LOG.info("TCP port 8012 is free.")

        # Preserve the proven V4 startup sequence exactly through creation
        # of the zero-ownership arm controller.
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
        LOG.info("TeleVuer server initialized.")

        arm_ik = G1_29_ArmIK()

        arm_ctrl = SafeArmSdkController(
            cache,
            initial_lowstate,
            joint_velocity_limit_rps=args.joint_target_speed_rps,
        )
        arm_ctrl.set_weight(0.0)

        # Initialize only the Inspire DDS I/O and deterministic open command.
        # The retargeting import and 60 Hz worker remain deferred until the
        # first valid Quest XR frame is observed.
        finger_ctrl = FingerController(
            hand_tracking,
            frequency_hz=args.finger_frequency,
            command_speed_per_s=args.finger_command_speed_per_s,
            minimum_command=args.finger_minimum_command,
            stable_tracking_frames=args.finger_stable_tracking_frames,
            tracking_fault_frames=args.finger_tracking_fault_frames,
            tracking_stale_s=args.finger_tracking_stale_s,
            feedback_stale_s=args.finger_state_stale_s,
            feedback_timeout_s=args.finger_state_timeout_s,
        )
        finger_ctrl.set_mode(FingerMode.OPEN)
        LOG.info(
            "Finger retargeting worker deferred until the first valid XR frame."
        )

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
        last_hand_alignment_reason: Optional[str] = None
        finger_worker_fault_reported = False
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
            hand_tracking.update(tele_data)
            finger_ctrl.service_deferred()

            xr_valid, xr_reason = tracking_ready(
                tv_wrapper,
                tele_data,
            )

            if xr_valid and not finger_ctrl.worker_started():
                try:
                    finger_ctrl.request_worker_start()
                except Exception as exc:
                    LOG.exception(
                        "Could not request deferred finger worker: %s",
                        exc,
                    )

            finger_status = finger_ctrl.snapshot()
            finger_worker_error = finger_status["worker_error"]
            if finger_worker_error is not None:
                shutdown_after_release = True
                if not finger_worker_fault_reported:
                    LOG.error(
                        "Finger worker failure requires controlled shutdown: %s",
                        finger_worker_error,
                    )
                    finger_worker_fault_reported = True

                if state == State.LOCOMOTION_READY:
                    arm_ctrl.set_weight(0.0)
                    graceful_release_completed = True
                    break
                if state in (State.WAITING_FOR_STOP, State.XR_ALIGNMENT):
                    state = State.LOCOMOTION_READY
                    arm_ctrl.set_weight(0.0)
                    graceful_release_completed = True
                    break

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
                    if gate_ready or args.allow_locomotion_during_xr:
                        begin_return(state_data, arm_ctrl, now)
                        state = State.RETURN_ARMS_HOME
                        LOG.info(
                            "Exit requested. Returning arms after fault."
                        )
                    else:
                        LOG.warning(
                            "Exit pending: stop the robot and keep R3 neutral."
                        )

            if TOGGLE_REQUESTED:
                TOGGLE_REQUESTED = False

                if state == State.LOCOMOTION_READY:
                    graceful_release_completed = False
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
                    if gate_ready or args.allow_locomotion_during_xr:
                        begin_return(state_data, arm_ctrl, now)
                        state = State.RETURN_ARMS_HOME
                        LOG.info(
                            "Returning arms after fault: %s.",
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
                and (
                    gate_ready
                    or args.allow_locomotion_during_xr
                )
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

                    hand_alignment_ready, hand_alignment_reason = (
                        finger_ctrl.alignment_ready()
                    )
                    alignment_ok = (
                        max(left_error, right_error)
                        <= args.start_align_m
                        and bias_magnitude
                        <= args.max_registration_shift_m
                        and hand_alignment_ready
                    )

                    if not hand_alignment_ready:
                        if hand_alignment_reason != last_hand_alignment_reason:
                            LOG.warning(
                                "Finger alignment unavailable: %s.",
                                hand_alignment_reason,
                            )
                            last_hand_alignment_reason = hand_alignment_reason
                    else:
                        last_hand_alignment_reason = None

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
                    if args.allow_locomotion_during_xr:
                        remote_fault = False
                        body_fault = False
                    else:
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
                elif finger_status["worker_error"] is not None:
                    fault_reason = (
                        "finger worker failed: "
                        f"{finger_status['worker_error']}"
                    )
                elif bool(finger_status["feedback_fault"]):
                    fault_reason = (
                        "Inspire state feedback stale: "
                        f"{float(finger_status['feedback_age_s']):.3f} s"
                    )

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
                            "Locomotion policy=%s.",
                            args.teleop_weight,
                            (
                                "concurrent"
                                if args.allow_locomotion_during_xr
                                else "stationary-only"
                            ),
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
                # Once handback starts, always finish it. Freezing at nonzero
                # ownership because of R3 input leaves the system in a worse
                # state than continuing the deterministic return-to-zero.
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
                # Ownership release is also uninterrupted once it begins.
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
                        "The R3 operator may move normally."
                    )

                    if shutdown_after_release:
                        LOG.info(
                            "Controlled handback complete. Exiting."
                        )
                        break

            finger_ctrl.set_mode(finger_mode_for_arm_state(state))
            finger_status = finger_ctrl.snapshot()

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
                raw_xr_wrist_sep_text = "unavailable"
                corrected_xr_wrist_sep_text = "unavailable"
                measured_wrist_sep_text = "unavailable"
                target_wrist_sep_text = "unavailable"
                published_wrist_sep_text = "unavailable"

                if (
                    pose_is_valid(
                        getattr(tele_data, "left_wrist_pose", None)
                    )
                    and pose_is_valid(
                        getattr(tele_data, "right_wrist_pose", None)
                    )
                ):
                    raw_left = np.asarray(
                        tele_data.left_wrist_pose,
                        dtype=float,
                    )
                    raw_right = np.asarray(
                        tele_data.right_wrist_pose,
                        dtype=float,
                    )
                    raw_xr_wrist_sep = float(
                        np.linalg.norm(
                            raw_left[:3, 3] - raw_right[:3, 3]
                        )
                    )
                    raw_xr_wrist_sep_text = (
                        f"{raw_xr_wrist_sep:.3f} m"
                    )

                if (
                    xr_state.guarded_left is not None
                    and xr_state.guarded_right is not None
                ):
                    corrected_xr_wrist_sep = float(
                        np.linalg.norm(
                            xr_state.guarded_left[:3, 3]
                            - xr_state.guarded_right[:3, 3]
                        )
                    )
                    corrected_xr_wrist_sep_text = (
                        f"{corrected_xr_wrist_sep:.3f} m"
                    )

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

                    try:
                        measured_left, measured_right = (
                            current_wrist_poses_from_fk(
                                arm_ik,
                                measured_q,
                            )
                        )
                        target_left, target_right = (
                            current_wrist_poses_from_fk(
                                arm_ik,
                                user_target_q,
                            )
                        )
                        published_left, published_right = (
                            current_wrist_poses_from_fk(
                                arm_ik,
                                published_user_q,
                            )
                        )

                        measured_wrist_sep = float(
                            np.linalg.norm(
                                measured_left[:3, 3]
                                - measured_right[:3, 3]
                            )
                        )
                        target_wrist_sep = float(
                            np.linalg.norm(
                                target_left[:3, 3]
                                - target_right[:3, 3]
                            )
                        )
                        published_wrist_sep = float(
                            np.linalg.norm(
                                published_left[:3, 3]
                                - published_right[:3, 3]
                            )
                        )

                        measured_wrist_sep_text = (
                            f"{measured_wrist_sep:.3f} m"
                        )
                        target_wrist_sep_text = (
                            f"{target_wrist_sep:.3f} m"
                        )
                        published_wrist_sep_text = (
                            f"{published_wrist_sep:.3f} m"
                        )
                    except Exception as exc:
                        measured_wrist_sep_text = f"FK error: {exc}"

                LOG.info(
                    "SNAPSHOT state=%s weight=%.3f writes=%d "
                    "lowstate_count=%d age=%.4f s gate=%s %.2f/%.2f s "
                    "XR=%s head_yaw=%s deg registration=%s "
                    "IK-target-vs-measured=%s "
                    "published-user-vs-measured=%s "
                    "IK-target-vs-published=%s "
                    "wrist-separation[raw-xr/corrected-xr/measured/target/published]="
                    "%s/%s/%s/%s/%s current_q=%s",
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
                    raw_xr_wrist_sep_text,
                    corrected_xr_wrist_sep_text,
                    measured_wrist_sep_text,
                    target_wrist_sep_text,
                    published_wrist_sep_text,
                    current_q_text,
                )

                LOG.info(
                    "FINGER SNAPSHOT mode=%s writes=%d tracking=%s "
                    "stable=%d faults=%d state-age=%.3f s feedback-fault=%s "
                    "worker-phase=%s worker-age=%.3f s loops=%d retargets=%d "
                    "raw L=%s R=%s normalized L=%s R=%s "
                    "command L=%s R=%s state=%s",
                    finger_status["mode"].name,
                    int(finger_status["write_count"]),
                    finger_status["tracking_reason"],
                    int(finger_status["stable_count"]),
                    int(finger_status["fault_count"]),
                    float(finger_status["feedback_age_s"]),
                    bool(finger_status["feedback_fault"]),
                    str(finger_status["worker_phase"]),
                    float(finger_status["worker_age_s"]),
                    int(finger_status["worker_loop_count"]),
                    int(finger_status["retarget_count"]),
                    np.array2string(
                        np.asarray(finger_status["raw_left"]), precision=4
                    ),
                    np.array2string(
                        np.asarray(finger_status["raw_right"]), precision=4
                    ),
                    np.array2string(
                        np.asarray(finger_status["normalized_left"]), precision=3
                    ),
                    np.array2string(
                        np.asarray(finger_status["normalized_right"]), precision=3
                    ),
                    np.array2string(
                        np.asarray(finger_status["current_left"]), precision=3
                    ),
                    np.array2string(
                        np.asarray(finger_status["current_right"]), precision=3
                    ),
                    (
                        "unavailable"
                        if finger_status["state"] is None
                        else np.array2string(
                            np.asarray(finger_status["state"]), precision=3
                        )
                    ),
                )

            if now - last_status >= 1.0 / args.status_hz:
                remote = metrics.remote or RemoteState()
                LOG.info(
                    "state=%-18s | weight=%.3f | gate=%-6s %.2f/%.2f | "
                    "dq max/rms=%s/%s | yaw=%s | "
                    "R3=%s/%s/%s/%s max=%s | XR=%s | guard=%s | "
                    "fingers=%s tracking=%s state-age=%.3f | "
                    "worker=%s age=%.3f writes=%d retargets=%d",
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
                    finger_status["mode"].name,
                    (
                        "ok"
                        if bool(finger_status["tracking_valid"])
                        else str(finger_status["tracking_reason"])
                    ),
                    float(finger_status["feedback_age_s"]),
                    str(finger_status["worker_phase"]),
                    float(finger_status["worker_age_s"]),
                    int(finger_status["write_count"]),
                    int(finger_status["retarget_count"]),
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

        if finger_ctrl is not None:
            try:
                LOG.warning("Final controlled hand opening in progress.")
                opened = finger_ctrl.request_open_and_wait(timeout_s=3.5)
                if not opened:
                    LOG.warning(
                        "Finger worker did not confirm fully-open commands "
                        "before timeout; sending final direct open samples."
                    )
                finger_ctrl.close()
                LOG.info("Both hand commands are fully open.")
            except Exception as exc:
                LOG.error("Finger controller close failed: %s", exc)

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

        kill_all_descendants()
        LOG.info("Exited.")


if __name__ == "__main__":
    raise SystemExit(main())
