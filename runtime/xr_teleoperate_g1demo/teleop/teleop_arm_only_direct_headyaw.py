#!/usr/bin/env python3
"""Direct head-yaw-relative XR teleoperation for Unitree G1 29-DoF arms.

Purpose
-------
Arm-only control. No locomotion, no end-effector control, no image server.

Data path
---------
Quest/OpenXR
    -> TeleVuer live ``head_yaw`` wrist poses
    -> raw-head / tracking validity checks
    -> single-frame jump guard
    -> Cartesian speed limiter
    -> G1 dual-arm IK
    -> simulator or robot arm controller

Keys
----
r : engage after matching your arms to the robot pose
p : print an unmistakable status snapshot
q : stop, return arms home, and exit

Important
---------
Use the locally served Quest page:
    https://<THINKCENTRE-IP>:8012/?ws=wss://<THINKCENTRE-IP>:8012

Do not use the hosted encoded ``https://vuer.ai?ws=...`` page for this setup.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pinocchio as pin
from sshkeyboard import listen_keyboard, stop_listening

# Make ``teleop.*`` imports work when this file is run from ~/xr_teleoperate/teleop.
CURRENT_DIR = Path(__file__).resolve().parent
PARENT_DIR = CURRENT_DIR.parent
if str(PARENT_DIR) not in sys.path:
    sys.path.append(str(PARENT_DIR))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
from teleop.utils.motion_switcher import MotionSwitcher


START = False
STOP = False
SNAPSHOT = False


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_direct_headyaw")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)-8s [Direct] %(message)s",
            "%H:%M:%S",
        )
    )
    logger.addHandler(handler)
    logger.propagate = False
    return logger


LOG = configure_logging()


def on_press(key: str) -> None:
    global START, STOP, SNAPSHOT

    if key == "r":
        if START:
            LOG.warning("[r] already engaged.")
        else:
            START = True
            LOG.info("[r] engage requested.")
    elif key == "p":
        SNAPSHOT = True
    elif key == "q":
        START = False
        STOP = True
        LOG.info("[q] stop requested.")
        try:
            stop_listening()
        except Exception:
            pass
    else:
        LOG.warning("Key %r has no action. Use r, p, or q.", key)


def pose_is_valid(pose: object) -> bool:
    try:
        value = np.asarray(pose, dtype=float)
    except Exception:
        return False

    if value.shape != (4, 4):
        return False
    if not np.isfinite(value).all():
        return False
    if np.linalg.norm(value) < 1e-8:
        return False
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-3):
        return False

    rotation = value[:3, :3]
    determinant = float(np.linalg.det(rotation))
    if not 0.8 < determinant < 1.2:
        return False
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-2):
        return False

    return True


def raw_head_pose(tv_wrapper: TeleVuerWrapper) -> Optional[np.ndarray]:
    """Return the raw OpenXR/Vuer headset pose, or None when unavailable."""
    try:
        value = np.asarray(tv_wrapper.tvuer.head_pose, dtype=float)
    except Exception:
        return None
    return value.copy() if pose_is_valid(value) else None


def tracking_frame_valid(
    tv_wrapper: TeleVuerWrapper,
    tele_data: object,
) -> tuple[bool, str]:
    """Validate the exact signals required for direct head-yaw control."""
    if not bool(getattr(tele_data, "motion_data_ready", False)):
        return False, "XR motion data is not ready"

    # This check intentionally rejects TeleVuer's fixed fallback head pose.
    # It prevents accidental control through the hosted vuer.ai page, where
    # hands may arrive while CAMERA_MOVE/head pose remains all zeros.
    if raw_head_pose(tv_wrapper) is None:
        return (
            False,
            "raw headset pose is missing; open the local-IP Vuer page, not vuer.ai",
        )

    if not pose_is_valid(getattr(tele_data, "head_pose", None)):
        return False, "processed headset pose is invalid"

    if not pose_is_valid(getattr(tele_data, "left_wrist_pose", None)):
        return False, "left wrist pose is invalid"

    if not pose_is_valid(getattr(tele_data, "right_wrist_pose", None)):
        return False, "right wrist pose is invalid"

    left_hand = getattr(tele_data, "left_hand_pos", None)
    right_hand = getattr(tele_data, "right_hand_pos", None)

    if left_hand is None or right_hand is None:
        return False, "hand joint arrays are unavailable"

    try:
        left_hand_array = np.asarray(left_hand, dtype=float)
        right_hand_array = np.asarray(right_hand, dtype=float)
    except Exception:
        return False, "hand joint arrays are not numeric"

    if not np.isfinite(left_hand_array).all() or not np.isfinite(right_hand_array).all():
        return False, "hand joint arrays contain NaN/Inf"

    if np.linalg.norm(left_hand_array) < 1e-6:
        return False, "left hand tracking is all zeros"

    if np.linalg.norm(right_hand_array) < 1e-6:
        return False, "right hand tracking is all zeros"

    return True, "ok"


def rotation_distance_deg(first_pose: np.ndarray, second_pose: np.ndarray) -> float:
    relative = first_pose[:3, :3].T @ second_pose[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def pose_jump_within(
    previous_pose: np.ndarray,
    candidate_pose: np.ndarray,
    max_translation: float,
    max_rotation_deg: float,
) -> bool:
    translation_jump = float(
        np.linalg.norm(candidate_pose[:3, 3] - previous_pose[:3, 3])
    )
    rotation_jump = rotation_distance_deg(previous_pose, candidate_pose)
    return (
        translation_jump <= max_translation
        and rotation_jump <= max_rotation_deg
    )


def update_guarded_pose(
    candidate_pose: np.ndarray,
    last_good_pose: Optional[np.ndarray],
    pending_pose: Optional[np.ndarray],
    pending_count: int,
    max_jump_m: float,
    max_jump_deg: float,
    reacquire_frames: int,
) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray], int, bool, bool]:
    """Reject one-frame jumps and accept a stable reacquired pose."""
    candidate_pose = candidate_pose.copy()

    if last_good_pose is None:
        return candidate_pose, candidate_pose, None, 0, False, False

    if pose_jump_within(
        last_good_pose,
        candidate_pose,
        max_jump_m,
        max_jump_deg,
    ):
        had_pending_jump = pending_count > 0
        return (
            candidate_pose,
            candidate_pose,
            None,
            0,
            False,
            had_pending_jump,
        )

    pending_translation_limit = max(max_jump_m * 0.30, 0.01)
    pending_rotation_limit = max(max_jump_deg * 0.30, 5.0)

    if (
        pending_pose is not None
        and pose_jump_within(
            pending_pose,
            candidate_pose,
            pending_translation_limit,
            pending_rotation_limit,
        )
    ):
        pending_count += 1
    else:
        pending_pose = candidate_pose.copy()
        pending_count = 1

    if pending_count >= reacquire_frames:
        return candidate_pose, candidate_pose, None, 0, False, True

    return (
        last_good_pose.copy(),
        last_good_pose,
        pending_pose,
        pending_count,
        True,
        False,
    )


def rate_limit_pose(
    previous_pose: Optional[np.ndarray],
    target_pose: np.ndarray,
    dt: float,
    max_linear_speed: float,
    max_angular_speed_deg: float,
) -> np.ndarray:
    """Rate-limit Cartesian translation and rotation before IK."""
    if previous_pose is None:
        return target_pose.copy()

    result = previous_pose.copy()

    translation_delta = target_pose[:3, 3] - previous_pose[:3, 3]
    translation_distance = float(np.linalg.norm(translation_delta))
    max_translation_step = max_linear_speed * dt

    if (
        translation_distance > max_translation_step
        and translation_distance > 1e-9
    ):
        translation_delta *= max_translation_step / translation_distance

    result[:3, 3] = previous_pose[:3, 3] + translation_delta

    rotation_delta = previous_pose[:3, :3].T @ target_pose[:3, :3]
    rotation_vector = np.asarray(pin.log3(rotation_delta), dtype=float).reshape(3)
    rotation_angle = float(np.linalg.norm(rotation_vector))
    max_rotation_step = math.radians(max_angular_speed_deg) * dt

    if rotation_angle > max_rotation_step and rotation_angle > 1e-9:
        rotation_vector *= max_rotation_step / rotation_angle

    result[:3, :3] = previous_pose[:3, :3] @ pin.exp3(rotation_vector)
    return result


def current_wrist_poses_from_fk(
    arm_ik: G1_29_ArmIK,
    arm_q: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return current G1 left/right end-effector poses in the IK frame."""
    q = np.asarray(arm_q, dtype=float).reshape(-1)
    model = arm_ik.reduced_robot.model
    data = arm_ik.reduced_robot.data

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    def frame_pose(frame_id: int) -> np.ndarray:
        placement = data.oMf[frame_id]
        pose = np.eye(4, dtype=float)
        pose[:3, :3] = placement.rotation
        pose[:3, 3] = placement.translation
        return pose

    return frame_pose(arm_ik.L_hand_id), frame_pose(arm_ik.R_hand_id)


def robot_yaw_deg(head_pose: np.ndarray) -> float:
    forward = np.asarray(head_pose[:3, 0], dtype=float).copy()
    forward[2] = 0.0
    norm = float(np.linalg.norm(forward))
    if norm < 1e-8:
        return float("nan")
    forward /= norm
    return math.degrees(math.atan2(float(forward[1]), float(forward[0])))


def fmt_position(pose: Optional[np.ndarray]) -> str:
    if pose is None:
        return "None"
    return np.array2string(
        np.asarray(pose[:3, 3]),
        precision=3,
        suppress_small=True,
        sign=" ",
    )


def print_snapshot(
    tv_wrapper: TeleVuerWrapper,
    tele_data: object,
    current_left: np.ndarray,
    current_right: np.ndarray,
    guarded_left: Optional[np.ndarray],
    guarded_right: Optional[np.ndarray],
    engaged: bool,
    guard_active: bool,
) -> None:
    raw = raw_head_pose(tv_wrapper)
    processed_head = (
        np.asarray(tele_data.head_pose, dtype=float)
        if pose_is_valid(getattr(tele_data, "head_pose", None))
        else None
    )
    left_target = (
        np.asarray(tele_data.left_wrist_pose, dtype=float)
        if pose_is_valid(getattr(tele_data, "left_wrist_pose", None))
        else None
    )
    right_target = (
        np.asarray(tele_data.right_wrist_pose, dtype=float)
        if pose_is_valid(getattr(tele_data, "right_wrist_pose", None))
        else None
    )

    LOG.info(
        "[Snapshot] engaged=%s guard_active=%s raw_head=%s head_yaw=%s",
        engaged,
        guard_active,
        "valid" if raw is not None else "INVALID",
        (
            f"{robot_yaw_deg(processed_head):+.1f} deg"
            if processed_head is not None
            else "INVALID"
        ),
    )
    LOG.info(
        "[Snapshot] current FK L=%s R=%s",
        fmt_position(current_left),
        fmt_position(current_right),
    )
    LOG.info(
        "[Snapshot] live target L=%s R=%s",
        fmt_position(left_target),
        fmt_position(right_target),
    )
    LOG.info(
        "[Snapshot] guarded target L=%s R=%s",
        fmt_position(guarded_left),
        fmt_position(guarded_right),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Direct live head-yaw-relative Unitree G1 arm teleoperation."
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=30.0,
        help="Control-loop frequency in Hz.",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Use the Isaac simulation DDS domain.",
    )
    parser.add_argument(
        "--network-interface",
        type=str,
        default=None,
        help="Optional DDS interface name.",
    )
    parser.add_argument(
        "--tracking-jump-m",
        type=float,
        default=0.10,
        help="Maximum accepted single-frame wrist translation jump.",
    )
    parser.add_argument(
        "--tracking-jump-deg",
        type=float,
        default=40.0,
        help="Maximum accepted single-frame wrist rotation jump.",
    )
    parser.add_argument(
        "--tracking-reacquire-frames",
        type=int,
        default=8,
        help="Stable frames required to accept a discontinuous reacquired pose.",
    )
    parser.add_argument(
        "--max-wrist-speed",
        type=float,
        default=0.20,
        help="Maximum Cartesian wrist-target speed in metres per second.",
    )
    parser.add_argument(
        "--max-wrist-rotation-speed-deg",
        type=float,
        default=90.0,
        help="Maximum wrist-target angular speed in degrees per second.",
    )
    parser.add_argument(
        "--start-align-m",
        type=float,
        default=0.20,
        help=(
            "Maximum position mismatch per wrist when direct control engages. "
            "Match your arms to the robot before pressing r."
        ),
    )
    parser.add_argument(
        "--stable-start-frames",
        type=int,
        default=20,
        help="Consecutive valid/aligned XR frames required after pressing r.",
    )
    parser.add_argument(
        "--status-hz",
        type=float,
        default=1.0,
        help="Compact status-report frequency; zero disables periodic status.",
    )
    return parser


def main() -> int:
    global SNAPSHOT

    args = build_parser().parse_args()

    if args.frequency <= 0:
        raise ValueError("--frequency must be positive")
    if args.tracking_jump_m <= 0:
        raise ValueError("--tracking-jump-m must be positive")
    if args.tracking_reacquire_frames < 1:
        raise ValueError("--tracking-reacquire-frames must be at least 1")
    if args.max_wrist_speed <= 0:
        raise ValueError("--max-wrist-speed must be positive")
    if args.max_wrist_rotation_speed_deg <= 0:
        raise ValueError("--max-wrist-rotation-speed-deg must be positive")
    if args.start_align_m <= 0:
        raise ValueError("--start-align-m must be positive")
    if args.stable_start_frames < 1:
        raise ValueError("--stable-start-frames must be at least 1")

    arm_ctrl: Optional[G1_29_ArmController] = None
    tv_wrapper: Optional[TeleVuerWrapper] = None
    keyboard_thread: Optional[threading.Thread] = None
    motion_switcher: Optional[MotionSwitcher] = None

    try:
        ChannelFactoryInitialize(
            1 if args.sim else 0,
            networkInterface=args.network_interface,
        )

        keyboard_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={
                "on_press": on_press,
                "until": None,
                "sequential": False,
            },
            daemon=True,
        )
        keyboard_thread.start()

        LOG.info("DIRECT_HEAD_YAW_ARM_ONLY_V1")
        LOG.info(
            "Mode=%s | no image server | no locomotion | no end effector",
            "SIM" if args.sim else "REAL",
        )
        LOG.info(
            "Open ONLY the local Quest page: "
            "https://<THINKCENTRE-IP>:8012/?ws=wss://<THINKCENTRE-IP>:8012"
        )
        LOG.info("Do not use the encoded hosted https://vuer.ai?ws=... page.")

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

        if not args.sim:
            motion_switcher = MotionSwitcher()
            status, _ = motion_switcher.Enter_Debug_Mode()
            LOG.info(
                "Enter debug mode: %s",
                "Success" if status == 0 else f"Failed ({status})",
            )

        arm_ik = G1_29_ArmIK()
        arm_ctrl = G1_29_ArmController(
            motion_mode=False,
            simulation_mode=args.sim,
        )

        LOG.info(
            "Tracking guard: %.2f m, %.1f deg, reacquire=%d frames.",
            args.tracking_jump_m,
            args.tracking_jump_deg,
            args.tracking_reacquire_frames,
        )
        LOG.info(
            "Cartesian limiter: %.2f m/s, %.1f deg/s.",
            args.max_wrist_speed,
            args.max_wrist_rotation_speed_deg,
        )
        LOG.info(
            "Match your arms to the robot pose, then press [r]. "
            "Use [p] for a labeled snapshot and [q] to exit."
        )

        while not START and not STOP:
            time.sleep(0.02)

        if STOP:
            return 0

        arm_ctrl.speed_gradual_max()

        engaged = False
        stable_start_count = 0
        last_start_log = 0.0
        last_invalid_reason: Optional[str] = None
        tracking_lost = False

        last_good_left: Optional[np.ndarray] = None
        last_good_right: Optional[np.ndarray] = None
        left_pending: Optional[np.ndarray] = None
        right_pending: Optional[np.ndarray] = None
        left_pending_count = 0
        right_pending_count = 0
        guarded_left: Optional[np.ndarray] = None
        guarded_right: Optional[np.ndarray] = None

        last_left_command: Optional[np.ndarray] = None
        last_right_command: Optional[np.ndarray] = None
        last_command_time = time.monotonic()
        last_status_time = 0.0
        guard_was_active = False

        while not STOP:
            loop_start = time.monotonic()

            tele_data = tv_wrapper.get_tele_data()
            current_q = arm_ctrl.get_current_dual_arm_q()
            current_dq = arm_ctrl.get_current_dual_arm_dq()
            current_left, current_right = current_wrist_poses_from_fk(
                arm_ik,
                current_q,
            )

            frame_valid, invalid_reason = tracking_frame_valid(
                tv_wrapper,
                tele_data,
            )

            if not frame_valid:
                stable_start_count = 0
                if invalid_reason != last_invalid_reason:
                    LOG.warning("Tracking not usable: %s.", invalid_reason)
                    last_invalid_reason = invalid_reason
                tracking_lost = True

                if SNAPSHOT:
                    SNAPSHOT = False
                    print_snapshot(
                        tv_wrapper,
                        tele_data,
                        current_left,
                        current_right,
                        guarded_left,
                        guarded_right,
                        engaged,
                        guard_was_active,
                    )

                sleep_time = max(
                    0.0,
                    (1.0 / args.frequency) - (time.monotonic() - loop_start),
                )
                time.sleep(sleep_time)
                continue

            if last_invalid_reason is not None:
                LOG.info("Valid local headset and hand tracking received.")
                last_invalid_reason = None

            live_left = np.asarray(tele_data.left_wrist_pose, dtype=float).copy()
            live_right = np.asarray(tele_data.right_wrist_pose, dtype=float).copy()

            if not engaged:
                left_start_error = float(
                    np.linalg.norm(live_left[:3, 3] - current_left[:3, 3])
                )
                right_start_error = float(
                    np.linalg.norm(live_right[:3, 3] - current_right[:3, 3])
                )
                max_start_error = max(left_start_error, right_start_error)

                now = time.monotonic()
                if max_start_error <= args.start_align_m:
                    stable_start_count += 1
                else:
                    stable_start_count = 0

                if now - last_start_log >= 1.0:
                    LOG.info(
                        "Start alignment: L=%.3f m R=%.3f m "
                        "(limit %.3f m), stable=%d/%d.",
                        left_start_error,
                        right_start_error,
                        args.start_align_m,
                        stable_start_count,
                        args.stable_start_frames,
                    )
                    last_start_log = now

                if stable_start_count < args.stable_start_frames:
                    if SNAPSHOT:
                        SNAPSHOT = False
                        print_snapshot(
                            tv_wrapper,
                            tele_data,
                            current_left,
                            current_right,
                            live_left,
                            live_right,
                            engaged,
                            False,
                        )

                    sleep_time = max(
                        0.0,
                        (1.0 / args.frequency) - (time.monotonic() - loop_start),
                    )
                    time.sleep(sleep_time)
                    continue

                # Direct mode: these are already current live head-yaw-relative
                # poses in the robot IK frame. No neutral offset is calculated.
                last_good_left = live_left.copy()
                last_good_right = live_right.copy()
                guarded_left = live_left.copy()
                guarded_right = live_right.copy()

                # Begin the limiter at the robot's actual current FK pose.
                last_left_command = current_left.copy()
                last_right_command = current_right.copy()
                last_command_time = time.monotonic()

                engaged = True
                tracking_lost = False
                LOG.info(
                    "ENGAGED: direct live head_yaw targets are now driving IK."
                )
                LOG.info(
                    "No frozen yaw, relative neutral, profile, body frame, "
                    "axis registration, or operator-yaw offset is active."
                )

            if tracking_lost:
                LOG.info(
                    "Tracking returned; validating continuity before accepting "
                    "a discontinuous pose."
                )
                tracking_lost = False

            (
                guarded_left,
                last_good_left,
                left_pending,
                left_pending_count,
                left_rejected,
                left_reacquired,
            ) = update_guarded_pose(
                live_left,
                last_good_left,
                left_pending,
                left_pending_count,
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
                right_reacquired,
            ) = update_guarded_pose(
                live_right,
                last_good_right,
                right_pending,
                right_pending_count,
                args.tracking_jump_m,
                args.tracking_jump_deg,
                args.tracking_reacquire_frames,
            )

            guard_active = left_rejected or right_rejected

            if guard_active and not guard_was_active:
                LOG.warning(
                    "Tracking jump held. L pending=%d, R pending=%d.",
                    left_pending_count,
                    right_pending_count,
                )
            elif not guard_active and guard_was_active:
                LOG.info("Tracking guard cleared.")

            if left_reacquired or right_reacquired:
                LOG.info(
                    "Stable tracking pose accepted after reacquisition."
                )

            guard_was_active = guard_active

            command_time = time.monotonic()
            command_dt = float(
                np.clip(command_time - last_command_time, 0.005, 0.10)
            )
            last_command_time = command_time

            left_command = rate_limit_pose(
                last_left_command,
                guarded_left,
                command_dt,
                args.max_wrist_speed,
                args.max_wrist_rotation_speed_deg,
            )
            right_command = rate_limit_pose(
                last_right_command,
                guarded_right,
                command_dt,
                args.max_wrist_speed,
                args.max_wrist_rotation_speed_deg,
            )

            last_left_command = left_command.copy()
            last_right_command = right_command.copy()

            sol_q, sol_tauff = arm_ik.solve_ik(
                left_command,
                right_command,
                current_q,
                current_dq,
            )
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            now = time.monotonic()
            periodic_status = (
                args.status_hz > 0
                and now - last_status_time >= 1.0 / args.status_hz
            )

            if periodic_status:
                target_error_left = float(
                    np.linalg.norm(guarded_left[:3, 3] - current_left[:3, 3])
                )
                target_error_right = float(
                    np.linalg.norm(guarded_right[:3, 3] - current_right[:3, 3])
                )
                LOG.info(
                    "RUN head_yaw=%+.1f deg | target error L/R=%.3f/%.3f m "
                    "| guard=%s",
                    robot_yaw_deg(np.asarray(tele_data.head_pose, dtype=float)),
                    target_error_left,
                    target_error_right,
                    "HOLD" if guard_active else "ok",
                )
                last_status_time = now

            if SNAPSHOT:
                SNAPSHOT = False
                print_snapshot(
                    tv_wrapper,
                    tele_data,
                    current_left,
                    current_right,
                    guarded_left,
                    guarded_right,
                    engaged,
                    guard_active,
                )

            sleep_time = max(
                0.0,
                (1.0 / args.frequency) - (time.monotonic() - loop_start),
            )
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        LOG.info("KeyboardInterrupt received.")
    except Exception:
        LOG.exception("Fatal error")
        return 1
    finally:
        if arm_ctrl is not None:
            try:
                LOG.info("Returning both arms home...")
                arm_ctrl.ctrl_dual_arm_go_home()
            except Exception as exc:
                LOG.error("Failed to return arms home: %s", exc)

        try:
            stop_listening()
        except Exception:
            pass

        if keyboard_thread is not None and keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)

        if tv_wrapper is not None:
            try:
                tv_wrapper.close()
            except Exception as exc:
                LOG.error("Failed to close TeleVuer: %s", exc)

        LOG.info("Exited.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
