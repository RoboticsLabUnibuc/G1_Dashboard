#!/usr/bin/env python3
"""Read-only locomotion <-> XR handover state-machine test for Unitree G1.

This program deliberately creates NO command publisher. It validates:

    locomotion ready
      -> request teleoperation
      -> embedded R3 sticks neutral
      -> G1 leg/waist motion settles
      -> XR wrist alignment + shared XYZ registration
      -> simulated arm-ownership ramp up
      -> simulated teleoperation active
      -> handback request
      -> simulated arm return
      -> simulated ownership ramp down
      -> locomotion ready

It does not move the robot's arms or legs.

Keys
----
c : request/cancel teleoperation, or request handback while active
p : print a detailed snapshot
q : quit

Quest page
----------
Use the local page:
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

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HgLowState
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

# Reuse the already validated tracking/FK/registration helpers.
from teleop_arm_only_direct_headyaw_registered_clutch import (
    apply_translation_bias,
    compute_shared_translation_bias,
    current_wrist_poses_from_fk,
    format_vector,
    pose_is_valid,
    raw_head_pose,
    robot_yaw_deg,
    tracking_frame_valid,
)


TOGGLE_REQUESTED = False
SNAPSHOT_REQUESTED = False
STOP = threading.Event()


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_locomotion_xr_dryrun")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)-8s [HandoverDryRun] %(message)s",
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
    SIM_RAMP_UP = auto()
    XR_ACTIVE = auto()
    SAFETY_FAULT_HOLD = auto()
    SIM_RETURN_ARMS = auto()
    SIM_RAMP_DOWN = auto()


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
        axes = np.asarray([self.lx, self.ly, self.rx, self.ry], dtype=float)
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
    remote: RemoteState = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.remote is None:
            self.remote = RemoteState()


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

    def snapshot(self) -> tuple[Optional[HgLowState], float, int]:
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


def keyboard_press(key: str) -> None:
    global TOGGLE_REQUESTED, SNAPSHOT_REQUESTED

    if key == "c":
        TOGGLE_REQUESTED = True
        LOG.info("[c] mode toggle requested.")
    elif key == "p":
        SNAPSHOT_REQUESTED = True
    elif key == "q":
        LOG.info("[q] quit requested.")
        STOP.set()
        try:
            stop_listening()
        except Exception:
            pass
    else:
        LOG.warning("Unknown key %r. Use c, p, or q.", key)


def signal_stop(signum: int, frame: object) -> None:
    del signum, frame
    STOP.set()
    try:
        stop_listening()
    except Exception:
        pass


def parse_embedded_remote(lowstate: HgLowState) -> RemoteState:
    """Decode the 40-byte R3 packet embedded in every hg LowState sample.

    Unitree layout:
      bytes 2..3   : uint16 button bitmask
      bytes 4..7   : float32 lx
      bytes 8..11  : float32 rx
      bytes 12..15 : float32 ry
      bytes 20..23 : float32 ly
    """
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
        all_dq = np.asarray(
            [float(lowstate.motor_state[i].dq) for i in range(29)],
            dtype=float,
        )
        leg_waist_dq = all_dq[:15]
        finite = leg_waist_dq[np.isfinite(leg_waist_dq)]

        if finite.size != 15:
            return MotionMetrics(lowstate_age_s=age_s)

        dq_max = float(np.max(np.abs(finite)))
        dq_rms = float(np.sqrt(np.mean(np.square(finite))))

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
            leg_waist_dq_max_rps=dq_max,
            leg_waist_dq_rms_rps=dq_rms,
            gyro_z_rps=float(gyro[2]),
            remote=remote,
        )
    except Exception:
        return MotionMetrics(lowstate_age_s=age_s)


def stop_gate_instant(
    metrics: MotionMetrics,
    args: argparse.Namespace,
) -> bool:
    if not metrics.valid:
        return False

    remote_neutral = (
        metrics.remote.max_abs_axis <= args.r3_deadband
    )
    body_quiet = (
        metrics.leg_waist_dq_rms_rps <= args.stop_dq_rms_rps
        and metrics.leg_waist_dq_max_rps <= args.stop_dq_hard_max_rps
        and abs(metrics.gyro_z_rps) <= args.stop_yaw_rate_rps
    )
    return remote_neutral and body_quiet


def definite_motion(
    metrics: MotionMetrics,
    args: argparse.Namespace,
) -> bool:
    if not metrics.valid:
        return True

    return (
        metrics.remote.max_abs_axis > args.active_r3_fault
        or metrics.leg_waist_dq_rms_rps > args.active_dq_rms_fault_rps
        or metrics.leg_waist_dq_max_rps > args.active_dq_max_fault_rps
        or abs(metrics.gyro_z_rps) > args.active_yaw_fault_rps
    )


def current_arm_q_dq(
    lowstate: HgLowState,
) -> tuple[np.ndarray, np.ndarray]:
    indices = list(range(15, 22)) + list(range(22, 29))
    q = np.asarray(
        [float(lowstate.motor_state[i].q) for i in indices],
        dtype=float,
    )
    dq = np.asarray(
        [float(lowstate.motor_state[i].dq) for i in indices],
        dtype=float,
    )
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only G1 locomotion/XR handover state-machine dry run."
        )
    )
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--network-interface", type=str, default=None)
    parser.add_argument("--frequency", type=float, default=30.0)
    parser.add_argument("--status-hz", type=float, default=2.0)
    parser.add_argument("--lowstate-fresh-s", type=float, default=0.10)

    parser.add_argument(
        "--r3-deadband",
        type=float,
        default=0.12,
    )
    parser.add_argument(
        "--stop-dq-rms-rps",
        type=float,
        default=0.18,
        help="RMS leg/waist joint-speed threshold for entering XR mode.",
    )
    parser.add_argument(
        "--stop-dq-hard-max-rps",
        type=float,
        default=0.75,
        help="Hard per-joint leg/waist speed ceiling for entering XR mode.",
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
        help=(
            "Continuous time for which R3-neutral and lower-body quiet "
            "conditions must remain true before XR alignment starts."
        ),
    )

    parser.add_argument(
        "--start-align-m",
        type=float,
        default=0.10,
        help="Maximum residual wrist mismatch after shared XYZ registration.",
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
        "--sim-ramp-up-s",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--sim-return-arms-s",
        type=float,
        default=4.0,
    )
    parser.add_argument(
        "--sim-ramp-down-s",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--active-r3-fault",
        type=float,
        default=0.18,
    )
    parser.add_argument(
        "--active-dq-rms-fault-rps",
        type=float,
        default=0.35,
    )
    parser.add_argument(
        "--active-dq-max-fault-rps",
        type=float,
        default=1.50,
    )
    parser.add_argument(
        "--active-yaw-fault-rps",
        type=float,
        default=0.18,
    )
    parser.add_argument(
        "--active-fault-frames",
        type=int,
        default=3,
    )
    parser.add_argument(
        "--tracking-fault-frames",
        type=int,
        default=3,
        help=(
            "Consecutive invalid XR frames before the dry-run enters the "
            "safety-fault hold state."
        ),
    )
    return parser


def main() -> int:
    global TOGGLE_REQUESTED, SNAPSHOT_REQUESTED

    args = build_parser().parse_args()

    for name in (
        "frequency",
        "status_hz",
        "lowstate_fresh_s",
        "r3_deadband",
        "stop_dq_rms_rps",
        "stop_dq_hard_max_rps",
        "stop_yaw_rate_rps",
        "stop_hold_s",
        "start_align_m",
        "max_registration_shift_m",
        "sim_ramp_up_s",
        "sim_return_arms_s",
        "sim_ramp_down_s",
    ):
        if getattr(args, name) <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")

    if args.stable_align_frames < 1:
        raise ValueError("--stable-align-frames must be at least 1")
    if args.active_fault_frames < 1:
        raise ValueError("--active-fault-frames must be at least 1")
    if args.tracking_fault_frames < 1:
        raise ValueError("--tracking-fault-frames must be at least 1")

    signal.signal(signal.SIGINT, signal_stop)
    signal.signal(signal.SIGTERM, signal_stop)

    keyboard_thread: Optional[threading.Thread] = None
    lowstate_subscriber: Optional[ChannelSubscriber] = None
    tv_wrapper: Optional[TeleVuerWrapper] = None

    try:
        ChannelFactoryInitialize(
            1 if args.sim else 0,
            networkInterface=args.network_interface,
        )

        cache = LowStateCache()
        lowstate_subscriber = ChannelSubscriber(
            "rt/lowstate",
            HgLowState,
        )
        lowstate_subscriber.Init(cache.callback, 1)

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

        LOG.info("G1_LOCOMOTION_XR_HANDOVER_DRYRUN_V2")
        LOG.info("READ ONLY: no command publisher is created.")
        LOG.info(
            "Mode=%s interface=%s",
            "SIM" if args.sim else "REAL",
            args.network_interface or "<automatic>",
        )
        LOG.info(
            "Stop gate: R3<=%.2f, leg/waist RMS<=%.2f rad/s, "
            "hard max<=%.2f rad/s, |yaw|<=%.2f rad/s for %.1f s.",
            args.r3_deadband,
            args.stop_dq_rms_rps,
            args.stop_dq_hard_max_rps,
            args.stop_yaw_rate_rps,
            args.stop_hold_s,
        )
        LOG.info(
            "Open the local Quest page only: "
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

        state = State.LOCOMOTION_READY
        state_started = time.monotonic()
        stop_quiet_since: Optional[float] = None
        align_stable_count = 0
        align_bias_sum = np.zeros(3, dtype=float)
        align_bias_samples = 0
        accepted_bias = np.zeros(3, dtype=float)
        active_fault_count = 0
        tracking_fault_count = 0
        safety_fault_reason = ""
        last_status = 0.0
        last_tracking_reason: Optional[str] = None

        LOG.info(
            "STATE LOCOMOTION_READY. R3 operator may move normally. "
            "When the robot has stopped and the sticks are neutral, press "
            "[c] to request XR mode."
        )
        LOG.info(
            "Dry-run sequence: stop check -> XR registration -> simulated "
            "weight ramp -> simulated XR active -> simulated handback."
        )

        while not STOP.is_set():
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
            xr_valid, xr_reason = tracking_ready(tv_wrapper, tele_data)

            if TOGGLE_REQUESTED:
                TOGGLE_REQUESTED = False

                if state == State.LOCOMOTION_READY:
                    state = State.WAITING_FOR_STOP
                    state_started = now
                    LOG.info(
                        "STATE WAITING_FOR_STOP. R3 sticks must be neutral "
                        "and the robot must settle."
                    )
                elif state in (
                    State.WAITING_FOR_STOP,
                    State.XR_ALIGNMENT,
                    State.SIM_RAMP_UP,
                ):
                    state = State.LOCOMOTION_READY
                    state_started = now
                    align_stable_count = 0
                    align_bias_sum[:] = 0.0
                    align_bias_samples = 0
                    LOG.info(
                        "XR request cancelled. STATE LOCOMOTION_READY."
                    )
                elif state == State.XR_ACTIVE:
                    state = State.SIM_RETURN_ARMS
                    state_started = now
                    LOG.info(
                        "HANDOVER REQUESTED. Live XR would freeze now. "
                        "SIMULATING controlled return to stock arm pose."
                    )
                elif state == State.SAFETY_FAULT_HOLD:
                    if gate_ready:
                        state = State.SIM_RETURN_ARMS
                        state_started = now
                        LOG.info(
                            "Robot is stationary. SIMULATING controlled arm "
                            "return after safety fault: %s.",
                            safety_fault_reason,
                        )
                    else:
                        LOG.warning(
                            "Still moving/unsettled; handback return has not "
                            "started."
                        )
                else:
                    LOG.warning(
                        "Transition already in progress; [c] ignored."
                    )

            if state == State.WAITING_FOR_STOP and gate_ready:
                state = State.XR_ALIGNMENT
                state_started = now
                align_stable_count = 0
                align_bias_sum[:] = 0.0
                align_bias_samples = 0
                LOG.info(
                    "FULL STOP CONFIRMED. STATE XR_ALIGNMENT. "
                    "VR operator: match both hands to the robot pose."
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
                            "Stop gate was lost during alignment; returning "
                            "to WAITING_FOR_STOP."
                        )
                elif not xr_valid:
                    align_stable_count = 0
                    align_bias_sum[:] = 0.0
                    align_bias_samples = 0
                    if xr_reason != last_tracking_reason:
                        LOG.warning("XR alignment unavailable: %s.", xr_reason)
                        last_tracking_reason = xr_reason
                else:
                    last_tracking_reason = None
                    arm_q, _ = current_arm_q_dq(lowstate)
                    robot_left, robot_right = current_wrist_poses_from_fk(
                        arm_ik,
                        arm_q,
                    )
                    live_left = np.asarray(
                        tele_data.left_wrist_pose,
                        dtype=float,
                    )
                    live_right = np.asarray(
                        tele_data.right_wrist_pose,
                        dtype=float,
                    )
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
                    bias_magnitude = float(np.linalg.norm(candidate_bias))

                    alignment_ok = (
                        max(left_error, right_error) <= args.start_align_m
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
                        align_stable_count >= args.stable_align_frames
                        and align_bias_samples > 0
                    ):
                        accepted_bias = (
                            align_bias_sum / float(align_bias_samples)
                        )
                        state = State.SIM_RAMP_UP
                        state_started = now
                        LOG.info(
                            "XR ALIGNMENT VALID. registration=%s. "
                            "SIMULATING ownership ramp 0 -> 1.",
                            format_vector(accepted_bias),
                        )

            if state == State.SIM_RAMP_UP:
                progress = min(
                    1.0,
                    (now - state_started) / args.sim_ramp_up_s,
                )
                if progress >= 1.0:
                    state = State.XR_ACTIVE
                    state_started = now
                    active_fault_count = 0
                    tracking_fault_count = 0
                    safety_fault_reason = ""
                    LOG.info(
                        "STATE XR_ACTIVE (DRY RUN). No arm command is being "
                        "sent. Press [c] to simulate handback."
                    )

            if state == State.XR_ACTIVE:
                if definite_motion(metrics, args):
                    active_fault_count += 1
                else:
                    active_fault_count = 0

                if xr_valid:
                    tracking_fault_count = 0
                else:
                    tracking_fault_count += 1

                movement_fault = (
                    active_fault_count >= args.active_fault_frames
                )
                tracking_fault = (
                    tracking_fault_count >= args.tracking_fault_frames
                )

                if movement_fault or tracking_fault:
                    state = State.SAFETY_FAULT_HOLD
                    state_started = now

                    if movement_fault:
                        safety_fault_reason = (
                            "lower-body motion or R3 input during XR mode"
                        )
                    else:
                        safety_fault_reason = (
                            f"XR tracking loss: {xr_reason}"
                        )

                    LOG.error(
                        "SAFETY FAULT: %s. In the future live controller, "
                        "the XR arm target would freeze while ownership is "
                        "retained. Restore safe conditions, then press [c] "
                        "to simulate the controlled handback.",
                        safety_fault_reason,
                    )

            if state == State.SIM_RETURN_ARMS:
                progress = min(
                    1.0,
                    (now - state_started) / args.sim_return_arms_s,
                )
                if progress >= 1.0:
                    state = State.SIM_RAMP_DOWN
                    state_started = now
                    LOG.info(
                        "SIMULATED arms reached stock pose. "
                        "SIMULATING ownership ramp 1 -> 0."
                    )

            if state == State.SIM_RAMP_DOWN:
                progress = min(
                    1.0,
                    (now - state_started) / args.sim_ramp_down_s,
                )
                if progress >= 1.0:
                    state = State.LOCOMOTION_READY
                    state_started = now
                    LOG.info(
                        "STATE LOCOMOTION_READY. Simulated arm ownership=0. "
                        "R3 operator may resume walking."
                    )

            if SNAPSHOT_REQUESTED:
                SNAPSHOT_REQUESTED = False
                head_yaw = "INVALID"
                if pose_is_valid(getattr(tele_data, "head_pose", None)):
                    head_yaw = (
                        f"{robot_yaw_deg(np.asarray(tele_data.head_pose, dtype=float)):+.1f}"
                    )
                LOG.info(
                    "SNAPSHOT state=%s lowstate_count=%d age=%.4f s "
                    "gate=%s %.2f/%.2f s XR=%s head_yaw=%s deg "
                    "registration=%s",
                    state.name,
                    lowstate_count,
                    metrics.lowstate_age_s,
                    "READY" if gate_ready else (
                        "timing" if gate_instant else "moving"
                    ),
                    gate_elapsed,
                    args.stop_hold_s,
                    xr_valid,
                    head_yaw,
                    format_vector(accepted_bias),
                )

            if now - last_status >= 1.0 / args.status_hz:
                remote = metrics.remote
                LOG.info(
                    "state=%-20s | gate=%-6s %.2f/%.2f s | "
                    "dq max/rms=%s/%s | yaw=%s | "
                    "R3=%s/%s/%s/%s max=%s | XR=%s",
                    state.name,
                    "READY" if gate_ready else (
                        "timing" if gate_instant else "moving"
                    ),
                    gate_elapsed,
                    args.stop_hold_s,
                    (
                        f"{metrics.leg_waist_dq_max_rps:.3f}"
                        if math.isfinite(metrics.leg_waist_dq_max_rps)
                        else "?"
                    ),
                    (
                        f"{metrics.leg_waist_dq_rms_rps:.3f}"
                        if math.isfinite(metrics.leg_waist_dq_rms_rps)
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
                )
                last_status = now

            sleep_time = max(
                0.0,
                (1.0 / args.frequency)
                - (time.monotonic() - loop_start),
            )
            STOP.wait(sleep_time)

        return 0

    except KeyboardInterrupt:
        LOG.info("KeyboardInterrupt.")
        return 0
    except Exception:
        LOG.exception("Fatal error")
        return 1
    finally:
        try:
            stop_listening()
        except Exception:
            pass

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

        LOG.info("Exited. No command was sent.")


if __name__ == "__main__":
    raise SystemExit(main())
