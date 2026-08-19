#!/usr/bin/env python3
"""Read-only TeleVuer head-event diagnostic.

No DDS, IK, simulator, or robot commands.

Keys:
  p  print current raw/processed head and wrist status
  q  quit
"""
from __future__ import annotations

import argparse
import inspect
import logging
import math
import threading
import time
from pathlib import Path

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening
import televuer
from televuer import TeleVuerWrapper

STOP = False
SNAPSHOT = False


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_head_event_probe")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)-8s %(message)s", "%H:%M:%S"))
    logger.addHandler(handler)
    logger.propagate = False
    return logger


LOG = configure_logging()


def on_press(key: str) -> None:
    global STOP, SNAPSHOT
    if key == "p":
        SNAPSHOT = True
    elif key == "q":
        STOP = True
        try:
            stop_listening()
        except Exception:
            pass
    else:
        LOG.warning("Key %r has no action. Use p or q.", key)


def pose_info(value: object) -> tuple[np.ndarray | None, str]:
    try:
        pose = np.asarray(value, dtype=float)
    except Exception as exc:
        return None, f"not array-like: {exc}"
    if pose.shape != (4, 4):
        return None, f"shape={pose.shape}, expected (4, 4)"
    if not np.isfinite(pose).all():
        return None, "contains NaN/Inf"
    norm = float(np.linalg.norm(pose))
    det = float(np.linalg.det(pose[:3, :3]))
    if norm < 1e-8:
        return None, "all-zero/uninitialized matrix"
    if abs(det) < 0.5:
        return None, f"singular rotation det={det:.6f}"
    return pose.copy(), f"valid det={det:.6f}"


def yaw_openxr_deg(pose: np.ndarray) -> float:
    # OpenXR basis: +x right, +y up, -z forward. Use projected forward (-z axis).
    forward = -pose[:3, 2].copy()
    forward[1] = 0.0
    n = float(np.linalg.norm(forward))
    if n < 1e-8:
        return float("nan")
    forward /= n
    return math.degrees(math.atan2(float(forward[0]), float(-forward[2])))


def yaw_robot_deg(pose: np.ndarray) -> float:
    # Robot basis: +x forward, +y left, +z up.
    forward = pose[:3, 0].copy()
    forward[2] = 0.0
    n = float(np.linalg.norm(forward))
    if n < 1e-8:
        return float("nan")
    forward /= n
    return math.degrees(math.atan2(float(forward[1]), float(forward[0])))


def fmt(v: np.ndarray) -> str:
    return np.array2string(np.asarray(v), precision=3, suppress_small=True, sign=" ")


def print_snapshot(index: int, tv: TeleVuerWrapper) -> None:
    try:
        data = tv.get_tele_data()
    except Exception as exc:
        LOG.exception("[Snapshot %d] get_tele_data failed: %s", index, exc)
        return

    raw_value = getattr(tv.tvuer, "head_pose", None)
    proc_value = getattr(data, "head_pose", None)
    left_value = getattr(data, "left_wrist_pose", None)
    right_value = getattr(data, "right_wrist_pose", None)

    raw, raw_status = pose_info(raw_value)
    proc, proc_status = pose_info(proc_value)
    left, left_status = pose_info(left_value)
    right, right_status = pose_info(right_value)
    ready = bool(getattr(data, "motion_data_ready", False))

    LOG.info("[Snapshot %d] motion_data_ready=%s", index, ready)
    if raw is None:
        LOG.error("[Snapshot %d] RAW head INVALID: %s", index, raw_status)
        try:
            arr = np.asarray(raw_value, dtype=float)
            LOG.info("[Snapshot %d] RAW head value=%s", index, fmt(arr))
        except Exception:
            LOG.info("[Snapshot %d] RAW head value=%r", index, raw_value)
    else:
        LOG.info(
            "[Snapshot %d] RAW head %s: pos=%s m, yaw(OpenXR)=%+.1f deg",
            index, raw_status, fmt(raw[:3, 3]), yaw_openxr_deg(raw),
        )

    if proc is None:
        LOG.error("[Snapshot %d] PROCESSED head INVALID: %s", index, proc_status)
    else:
        LOG.info(
            "[Snapshot %d] PROCESSED head %s: pos=%s m, yaw(robot)=%+.1f deg",
            index, proc_status, fmt(proc[:3, 3]), yaw_robot_deg(proc),
        )

    if left is None:
        LOG.error("[Snapshot %d] LEFT wrist INVALID: %s", index, left_status)
    else:
        LOG.info("[Snapshot %d] LEFT wrist %s: pos=%s m", index, left_status, fmt(left[:3, 3]))

    if right is None:
        LOG.error("[Snapshot %d] RIGHT wrist INVALID: %s", index, right_status)
    else:
        LOG.info("[Snapshot %d] RIGHT wrist %s: pos=%s m", index, right_status, fmt(right[:3, 3]))

    if raw is None and left is not None and right is not None:
        LOG.error(
            "[Snapshot %d] Hands are arriving but the raw headset matrix is not. "
            "TeleVuer cannot make head-yaw-relative wrist poses without CAMERA_MOVE/head-pose data.",
            index,
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-mode", choices=["hand", "controller"], default="hand")
    parser.add_argument("--display-mode", choices=["immersive", "ego", "pass-through"], default="pass-through")
    args = parser.parse_args()

    LOG.info("[Probe] HEAD_EVENT_PROBE_V1 -- read-only")
    LOG.info("[Probe] televuer module: %s", Path(televuer.__file__).resolve())
    LOG.info("[Probe] TeleVuerWrapper signature: %s", inspect.signature(TeleVuerWrapper))

    tv = TeleVuerWrapper(
        use_hand_tracking=args.input_mode == "hand",
        binocular=False,
        img_shape=(480, 640),
        display_mode=args.display_mode,
        zmq=False,
        webrtc=False,
        webrtc_url=None,
        arm_reference_mode="head_yaw",
    )

    thread = threading.Thread(
        target=listen_keyboard,
        kwargs={"on_press": on_press, "until": None, "sequential": False},
        daemon=True,
    )
    thread.start()

    LOG.info("[Probe] Enter VR, then press [p] at each test pose. Press [q] to quit.")
    index = 0
    global SNAPSHOT
    try:
        while not STOP:
            if SNAPSHOT:
                SNAPSHOT = False
                index += 1
                print_snapshot(index, tv)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_listening()
        except Exception:
            pass
        try:
            tv.close()
        except Exception:
            pass
    LOG.info("[Probe] Exited. No robot command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
