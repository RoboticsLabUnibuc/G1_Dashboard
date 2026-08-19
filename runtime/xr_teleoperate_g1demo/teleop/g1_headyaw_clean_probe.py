#!/usr/bin/env python3
"""Read-only probe for TeleVuer's live head-yaw-relative wrist poses.

This script does not initialize DDS, IK, Isaac Sim, or a robot controller.
It starts the Vuer/WebXR endpoint, reads processed wrist poses from
TeleVuer with arm_reference_mode='head_yaw', and reports how much those
poses change relative to a baseline captured with [r].

Keys:
  r  capture/re-capture a stable baseline
  p  print an immediate snapshot
  q  quit
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import threading
import time
from dataclasses import dataclass

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening
from televuer import TeleVuerWrapper


STOP = False
CAPTURE_BASELINE = False
PRINT_SNAPSHOT = False


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_headyaw_probe")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            fmt="%(asctime)s %(levelname)-8s %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    logger.addHandler(handler)
    logger.propagate = False
    return logger


LOGGER = configure_logging()


def on_press(key: str) -> None:
    global STOP, CAPTURE_BASELINE, PRINT_SNAPSHOT

    if key == "r":
        CAPTURE_BASELINE = True
    elif key == "p":
        PRINT_SNAPSHOT = True
    elif key == "q":
        STOP = True
        try:
            stop_listening()
        except Exception:
            pass
    else:
        LOGGER.warning("Key %r has no action. Use r, p, or q.", key)


def pose_is_valid(pose: object) -> bool:
    try:
        matrix = np.asarray(pose, dtype=float)
    except Exception:
        return False

    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        return False

    # TeleVuer uses an all-zero matrix when wrist tracking is invalid.
    if np.linalg.norm(matrix) < 1e-8:
        return False

    rotation = matrix[:3, :3]
    if abs(np.linalg.det(rotation)) < 0.5:
        return False

    return True


def rotation_delta_deg(reference: np.ndarray, current: np.ndarray) -> float:
    relative = reference[:3, :3].T @ current[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def yaw_deg(pose: np.ndarray) -> float:
    rotation = pose[:3, :3]
    return math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))


def format_vec(vector: np.ndarray) -> str:
    return np.array2string(
        np.asarray(vector, dtype=float),
        precision=3,
        suppress_small=True,
        sign=" ",
    )


@dataclass
class Baseline:
    left: np.ndarray
    right: np.ndarray
    head: np.ndarray | None


def stable_capture(
    tv_wrapper: TeleVuerWrapper,
    frame_count: int,
    max_position_step_m: float,
    max_rotation_step_deg: float,
) -> Baseline | None:
    """Capture a short stable average of both processed wrist poses."""

    left_samples: list[np.ndarray] = []
    right_samples: list[np.ndarray] = []
    head_samples: list[np.ndarray] = []
    last_warning = 0.0

    LOGGER.info(
        "[Probe] Hold the robot-matching arm pose still while the live "
        "head-yaw-relative baseline is captured..."
    )

    while not STOP:
        tele_data = tv_wrapper.get_tele_data()
        left = np.asarray(tele_data.left_wrist_pose, dtype=float)
        right = np.asarray(tele_data.right_wrist_pose, dtype=float)
        head = np.asarray(tele_data.head_pose, dtype=float)

        if not pose_is_valid(left) or not pose_is_valid(right):
            left_samples.clear()
            right_samples.clear()
            head_samples.clear()
            now = time.monotonic()
            if now - last_warning >= 1.0:
                LOGGER.warning("[Probe] Waiting for valid left and right wrist tracking.")
                last_warning = now
            time.sleep(0.02)
            continue

        if left_samples:
            position_step = max(
                float(np.linalg.norm(left[:3, 3] - left_samples[-1][:3, 3])),
                float(np.linalg.norm(right[:3, 3] - right_samples[-1][:3, 3])),
            )
            rotation_step = max(
                rotation_delta_deg(left_samples[-1], left),
                rotation_delta_deg(right_samples[-1], right),
            )

            if (
                position_step > max_position_step_m
                or rotation_step > max_rotation_step_deg
            ):
                left_samples.clear()
                right_samples.clear()
                head_samples.clear()

        left_samples.append(left.copy())
        right_samples.append(right.copy())
        if pose_is_valid(head):
            head_samples.append(head.copy())

        if len(left_samples) >= frame_count:
            # Average translations. Keep the final orientation because averaging
            # rotation matrices directly would not preserve orthonormality.
            left_baseline = left_samples[-1].copy()
            right_baseline = right_samples[-1].copy()
            left_baseline[:3, 3] = np.mean(
                [pose[:3, 3] for pose in left_samples], axis=0
            )
            right_baseline[:3, 3] = np.mean(
                [pose[:3, 3] for pose in right_samples], axis=0
            )

            head_baseline = head_samples[-1].copy() if head_samples else None

            return Baseline(
                left=left_baseline,
                right=right_baseline,
                head=head_baseline,
            )

        time.sleep(0.02)

    return None


def log_snapshot(baseline: Baseline, tele_data: object) -> None:
    left = np.asarray(tele_data.left_wrist_pose, dtype=float)
    right = np.asarray(tele_data.right_wrist_pose, dtype=float)
    head = np.asarray(tele_data.head_pose, dtype=float)

    left_delta = left[:3, 3] - baseline.left[:3, 3]
    right_delta = right[:3, 3] - baseline.right[:3, 3]
    mean_delta = 0.5 * (left_delta + right_delta)

    baseline_separation = baseline.left[:3, 3] - baseline.right[:3, 3]
    current_separation = left[:3, 3] - right[:3, 3]
    separation_delta = current_separation - baseline_separation

    left_rotation = rotation_delta_deg(baseline.left, left)
    right_rotation = rotation_delta_deg(baseline.right, right)
    max_translation = max(
        float(np.linalg.norm(left_delta)),
        float(np.linalg.norm(right_delta)),
    )

    head_yaw_text = "n/a"
    if pose_is_valid(head):
        head_yaw_text = f"{yaw_deg(head):+.1f} deg"

    LOGGER.info(
        "[Probe] mean d=%s m | L d=%s m | R d=%s m | "
        "separation d=%s m | max d=%.3f m | L/R rot=%.1f/%.1f deg | "
        "head-pose yaw=%s",
        format_vec(mean_delta),
        format_vec(left_delta),
        format_vec(right_delta),
        format_vec(separation_delta),
        max_translation,
        left_rotation,
        right_rotation,
        head_yaw_text,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only TeleVuer head-yaw coordinate probe"
    )
    parser.add_argument(
        "--input-mode",
        choices=["hand", "controller"],
        default="hand",
    )
    parser.add_argument(
        "--display-mode",
        choices=["immersive", "ego", "pass-through"],
        default="pass-through",
    )
    parser.add_argument(
        "--report-hz",
        type=float,
        default=2.0,
        help="Continuous report frequency after baseline capture",
    )
    parser.add_argument(
        "--capture-frames",
        type=int,
        default=20,
        help="Stable frames required for baseline capture",
    )
    args = parser.parse_args()

    if args.report_hz <= 0:
        parser.error("--report-hz must be greater than zero")
    if args.capture_frames < 3:
        parser.error("--capture-frames must be at least 3")

    # This matches the user's pass-through/no-image-server workflow. TeleVuer
    # supplies poses only; this probe does not initialize robot or simulator IO.
    tv_wrapper = TeleVuerWrapper(
        use_hand_tracking=args.input_mode == "hand",
        binocular=False,
        img_shape=(480, 640),
        display_mode=args.display_mode,
        zmq=False,
        webrtc=False,
        webrtc_url=None,
        arm_reference_mode="head_yaw",
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

    LOGGER.info("[Probe] CLEAN_HEAD_YAW_PROBE_V1")
    LOGGER.info("[Probe] Read-only: no DDS, IK, simulator, or robot commands.")
    LOGGER.info("[Probe] Enter VR in the Quest browser, then press [r].")
    LOGGER.info("[Probe] [r] baseline, [p] snapshot, [q] quit.")

    baseline: Baseline | None = None
    last_report = 0.0
    last_tracking_warning = 0.0

    global CAPTURE_BASELINE, PRINT_SNAPSHOT

    try:
        while not STOP:
            if CAPTURE_BASELINE:
                CAPTURE_BASELINE = False
                baseline = stable_capture(
                    tv_wrapper,
                    frame_count=args.capture_frames,
                    max_position_step_m=0.025,
                    max_rotation_step_deg=5.0,
                )
                if baseline is not None:
                    LOGGER.info(
                        "[Probe] Baseline captured. L=%s m, R=%s m.",
                        format_vec(baseline.left[:3, 3]),
                        format_vec(baseline.right[:3, 3]),
                    )
                    LOGGER.info(
                        "[Probe] Keep the same arm pose relative to your torso, "
                        "then turn or walk. Near-zero deltas mean the live "
                        "head-yaw frame is behaving correctly."
                    )
                    PRINT_SNAPSHOT = True

            tele_data = tv_wrapper.get_tele_data()
            left_valid = pose_is_valid(tele_data.left_wrist_pose)
            right_valid = pose_is_valid(tele_data.right_wrist_pose)

            if not left_valid or not right_valid:
                now = time.monotonic()
                if now - last_tracking_warning >= 1.0:
                    LOGGER.warning("[Probe] Wrist tracking unavailable; no sample reported.")
                    last_tracking_warning = now
                time.sleep(0.02)
                continue

            now = time.monotonic()
            due = baseline is not None and now - last_report >= 1.0 / args.report_hz
            if baseline is not None and (due or PRINT_SNAPSHOT):
                PRINT_SNAPSHOT = False
                log_snapshot(baseline, tele_data)
                last_report = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        LOGGER.info("[Probe] Keyboard interrupt; exiting.")
    finally:
        try:
            stop_listening()
        except Exception:
            pass

    LOGGER.info("[Probe] Exited. No robot command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
