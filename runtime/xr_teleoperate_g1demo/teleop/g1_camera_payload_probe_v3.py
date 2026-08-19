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
import os
import math
import threading
import time
from pathlib import Path

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening
import televuer
from televuer import TeleVuerWrapper
from televuer.televuer import TeleVuer


def _extract_camera_matrix(value: object) -> tuple[np.ndarray | None, str]:
    """Accept the payload layouts used by different Vuer releases."""
    candidates: list[tuple[object, str]] = []

    if isinstance(value, dict):
        camera = value.get("camera")
        if isinstance(camera, dict):
            candidates.append((camera.get("matrix"), 'event.value["camera"]["matrix"]'))
        candidates.append((value.get("matrix"), 'event.value["matrix"]'))

        # Last-resort recursive search for a field named "matrix".
        stack: list[tuple[object, str]] = [(value, "event.value")]
        while stack:
            obj, path = stack.pop()
            if isinstance(obj, dict):
                for key, child in obj.items():
                    child_path = f'{path}[{key!r}]'
                    if str(key).lower() == "matrix":
                        candidates.append((child, child_path))
                    elif isinstance(child, (dict, list, tuple)):
                        stack.append((child, child_path))
            elif isinstance(obj, (list, tuple)):
                for index, child in enumerate(obj):
                    if isinstance(child, (dict, list, tuple)):
                        stack.append((child, f"{path}[{index}]"))

    for candidate, path in candidates:
        if candidate is None:
            continue
        try:
            array = np.asarray(candidate, dtype=float)
        except Exception:
            continue
        if array.size != 16 or not np.isfinite(array).all():
            continue
        return array.reshape(4, 4), path

    return None, "<no 4x4 matrix found>"


async def patched_on_cam_move(self, event, session, fps=60):
    """Diagnostic replacement for TeleVuer.on_cam_move.

    The installed TeleVuer only accepts event.value["camera"]["matrix"] and
    silently suppresses all parser exceptions. Vuer camera events commonly
    expose the matrix directly as event.value["matrix"].
    """
    count = int(getattr(self, "_camera_payload_probe_count", 0)) + 1
    self._camera_payload_probe_count = count

    value = getattr(event, "value", None)
    matrix, source_path = _extract_camera_matrix(value)

    if count == 1:
        if isinstance(value, dict):
            print(
                "[CAMERA_MOVE probe] first event received; "
                f"event.key={getattr(event, 'key', None)!r}, "
                f"value keys={list(value.keys())!r}",
                flush=True,
            )
        else:
            print(
                "[CAMERA_MOVE probe] first event received; "
                f"event.key={getattr(event, 'key', None)!r}, "
                f"value type={type(value).__name__}",
                flush=True,
            )

    if matrix is None:
        if count <= 5 or count % 120 == 0:
            print(
                f"[CAMERA_MOVE probe] event #{count}: no usable 4x4 matrix found",
                flush=True,
            )
        return

    with self.head_pose_shared.get_lock():
        self.head_pose_shared[:] = matrix.reshape(-1).tolist()

    if count <= 3 or count % 120 == 0:
        print(
            f"[CAMERA_MOVE probe] event #{count}: head matrix updated from "
            f"{source_path}; position={np.array2string(matrix[:3, 3], precision=3)}",
            flush=True,
        )


# Patch before TeleVuerWrapper constructs TeleVuer and registers CAMERA_MOVE.
TeleVuer.on_cam_move = patched_on_cam_move

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

    LOG.info("[Probe] CAMERA_PAYLOAD_PROBE_V3 -- read-only temporary parser patch")
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

    LOG.info("[Probe] Enter VR. Press [p] at 0 deg, after turning, and after walking. Press [q] to quit.")
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
