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
from televuer.televuer import TeleVuer


_ORIGINAL_ON_HAND_MOVE = TeleVuer.on_hand_move
_ORIGINAL_ON_CAM_MOVE = TeleVuer.on_cam_move


def _describe_payload(value: object, max_depth: int = 4) -> list[str]:
    """Return a compact recursive description of an event payload."""
    lines: list[str] = []
    seen: set[int] = set()

    def walk(obj: object, path: str, depth: int) -> None:
        if depth > max_depth:
            return
        obj_id = id(obj)
        if obj_id in seen:
            return
        if isinstance(obj, (dict, list, tuple)):
            seen.add(obj_id)

        if isinstance(obj, dict):
            keys = list(obj.keys())
            lines.append(f"{path}: dict keys={keys!r}")
            for key, child in obj.items():
                child_path = f"{path}[{key!r}]"
                if isinstance(child, (dict, list, tuple)):
                    walk(child, child_path, depth + 1)
                else:
                    try:
                        arr = np.asarray(child)
                        if arr.ndim > 0:
                            lines.append(
                                f"{child_path}: {type(child).__name__} "
                                f"shape={arr.shape} dtype={arr.dtype}"
                            )
                        else:
                            text = repr(child)
                            if len(text) > 160:
                                text = text[:157] + "..."
                            lines.append(f"{child_path}: {type(child).__name__}={text}")
                    except Exception:
                        text = repr(child)
                        if len(text) > 160:
                            text = text[:157] + "..."
                        lines.append(f"{child_path}: {type(child).__name__}={text}")
        elif isinstance(obj, (list, tuple)):
            lines.append(f"{path}: {type(obj).__name__} len={len(obj)}")
            for index, child in enumerate(obj[:8]):
                child_path = f"{path}[{index}]"
                if isinstance(child, (dict, list, tuple)):
                    walk(child, child_path, depth + 1)
                else:
                    try:
                        arr = np.asarray(child)
                        lines.append(
                            f"{child_path}: {type(child).__name__} "
                            f"shape={arr.shape} dtype={arr.dtype}"
                        )
                    except Exception:
                        lines.append(f"{child_path}: {type(child).__name__}")
        else:
            lines.append(f"{path}: {type(obj).__name__}={obj!r}")

    walk(value, "event.value", 0)
    return lines


def _find_matrix_candidates(value: object, max_depth: int = 6) -> list[tuple[str, tuple[int, ...], str]]:
    """Find array-like values that could contain pose matrices."""
    found: list[tuple[str, tuple[int, ...], str]] = []
    seen: set[int] = set()

    def walk(obj: object, path: str, depth: int) -> None:
        if depth > max_depth:
            return
        obj_id = id(obj)
        if obj_id in seen:
            return
        if isinstance(obj, (dict, list, tuple)):
            seen.add(obj_id)

        if isinstance(obj, dict):
            for key, child in obj.items():
                walk(child, f"{path}[{key!r}]", depth + 1)
            return

        if isinstance(obj, (list, tuple)):
            try:
                arr = np.asarray(obj)
                if arr.dtype != object and arr.size in (16, 32, 48, 64):
                    found.append((path, tuple(arr.shape), str(arr.dtype)))
            except Exception:
                pass
            for index, child in enumerate(obj[:16]):
                if isinstance(child, (dict, list, tuple)):
                    walk(child, f"{path}[{index}]", depth + 1)
            return

        try:
            arr = np.asarray(obj)
            if arr.ndim > 0 and arr.size in (16, 32, 48, 64):
                found.append((path, tuple(arr.shape), str(arr.dtype)))
        except Exception:
            pass

    walk(value, "event.value", 0)
    return found


async def _patched_on_hand_move(self, event, session, *args, **kwargs):
    count = int(getattr(self, "_hand_payload_probe_count", 0)) + 1
    self._hand_payload_probe_count = count

    if count == 1:
        print(
            "[HAND_MOVE probe] first event received; "
            f"event.key={getattr(event, 'key', None)!r}, "
            f"event.type={getattr(event, 'etype', getattr(event, 'type', None))!r}",
            flush=True,
        )
        for line in _describe_payload(getattr(event, "value", None)):
            print(f"[HAND_MOVE payload] {line}", flush=True)
        candidates = _find_matrix_candidates(getattr(event, "value", None))
        if candidates:
            for path, shape, dtype in candidates:
                print(
                    f"[HAND_MOVE matrix candidate] {path}: shape={shape}, dtype={dtype}",
                    flush=True,
                )
        else:
            print("[HAND_MOVE probe] no 16/32/48/64-value matrix candidates found", flush=True)

    return await _ORIGINAL_ON_HAND_MOVE(self, event, session, *args, **kwargs)


async def _patched_on_cam_move(self, event, session, *args, **kwargs):
    count = int(getattr(self, "_camera_event_probe_count", 0)) + 1
    self._camera_event_probe_count = count
    if count == 1:
        print(
            "[CAMERA_MOVE probe] first event received unexpectedly; "
            f"value={getattr(event, 'value', None)!r}",
            flush=True,
        )
    return await _ORIGINAL_ON_CAM_MOVE(self, event, session, *args, **kwargs)


# Patch before TeleVuerWrapper creates TeleVuer and registers handlers.
TeleVuer.on_hand_move = _patched_on_hand_move
TeleVuer.on_cam_move = _patched_on_cam_move

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

    LOG.info("[Probe] HAND_PAYLOAD_PROBE_V4 -- read-only")
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

    LOG.info("[Probe] Enter VR and keep both hands visible. Wait for the HAND_MOVE payload dump, then press [p]. Press [q] to quit.")
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
