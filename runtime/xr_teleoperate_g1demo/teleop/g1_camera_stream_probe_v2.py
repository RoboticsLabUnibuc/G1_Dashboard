#!/usr/bin/env python3
"""Read-only Vuer/TeleVuer camera-stream diagnostic.

Purpose:
  * explicitly enables main-camera streaming with OrbitControls(stream=True)
  * logs every CAMERA_MOVE payload shape seen by TeleVuer
  * accepts both known payload layouts:
        event.value["camera"]["matrix"]
        event.value["matrix"]
  * prints raw and processed head status on [p]

No DDS, IK, simulator, or robot commands.

Keys:
  p  print snapshot
  q  quit
"""
from __future__ import annotations

import argparse
import asyncio
import importlib.metadata
import logging
import math
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening
import televuer
from televuer import TeleVuerWrapper
from televuer.televuer import TeleVuer
import inspect
import vuer.schemas as vuer_schemas
from vuer.schemas import Hands, MotionControllers

STOP = False
SNAPSHOT = False


def resolve_camera_control_schema():
    """Find the main-camera control schema across Vuer releases."""
    preferred = (
        "OrbitControls",
        "SceneCameraControl",
        "CameraControl",
    )
    for name in preferred:
        cls = getattr(vuer_schemas, name, None)
        if cls is not None:
            return cls, name
    return None, None


CAMERA_CONTROL_CLASS, CAMERA_CONTROL_NAME = resolve_camera_control_schema()


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_camera_stream_probe")
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


def describe(value: Any, depth: int = 0) -> str:
    """Compact, non-recursive description safe for event payloads."""
    if depth > 2:
        return type(value).__name__
    if isinstance(value, dict):
        parts = []
        for key, item in list(value.items())[:12]:
            parts.append(f"{key}:{describe(item, depth + 1)}")
        suffix = ",..." if len(value) > 12 else ""
        return "{" + ",".join(parts) + suffix + "}"
    if isinstance(value, (list, tuple)):
        if len(value) in (16, 4):
            return f"{type(value).__name__}[len={len(value)}]"
        return f"{type(value).__name__}[len={len(value)}]"
    if isinstance(value, np.ndarray):
        return f"ndarray{value.shape}"
    return type(value).__name__


def extract_matrix(value: Any) -> tuple[Any | None, str]:
    """Support old nested and newer top-level CAMERA_MOVE payload layouts."""
    if isinstance(value, dict):
        camera = value.get("camera")
        if isinstance(camera, dict) and camera.get("matrix") is not None:
            return camera.get("matrix"), 'event.value["camera"]["matrix"]'
        if camera is not None and getattr(camera, "matrix", None) is not None:
            return getattr(camera, "matrix"), 'event.value["camera"].matrix'
        if value.get("matrix") is not None:
            return value.get("matrix"), 'event.value["matrix"]'
    if getattr(value, "camera", None) is not None:
        camera = getattr(value, "camera")
        if getattr(camera, "matrix", None) is not None:
            return getattr(camera, "matrix"), "event.value.camera.matrix"
    if getattr(value, "matrix", None) is not None:
        return getattr(value, "matrix"), "event.value.matrix"
    return None, "no recognized matrix field"


async def patched_on_cam_move(self: TeleVuer, event: Any, session: Any, fps: int = 60) -> None:
    try:
        etype = getattr(event, "etype", None)
        key = getattr(event, "key", None)
        value = getattr(event, "value", None)
        print(
            f"[CAMERA_MOVE] received etype={etype!r} key={key!r} payload={describe(value)}",
            flush=True,
        )
        matrix_value, source = extract_matrix(value)
        if matrix_value is None:
            print(f"[CAMERA_MOVE] no matrix extracted: {source}", flush=True)
            return
        flat = np.asarray(matrix_value, dtype=float).reshape(-1)
        if flat.size != 16 or not np.isfinite(flat).all():
            print(
                f"[CAMERA_MOVE] rejected {source}: size={flat.size}, finite={np.isfinite(flat).all()}",
                flush=True,
            )
            return
        with self.head_pose_shared.get_lock():
            self.head_pose_shared[:] = flat.tolist()
        print(f"[CAMERA_MOVE] head matrix updated from {source}", flush=True)
    except Exception as exc:
        print(f"[CAMERA_MOVE] handler exception: {type(exc).__name__}: {exc}", flush=True)


async def patched_main_pass_through(self: TeleVuer, session: Any) -> None:
    if self.use_hand_tracking:
        session.upsert(
            Hands(stream=True, key="hands", hideLeft=True, hideRight=True),
            to="bgChildren",
        )
    else:
        session.upsert(
            MotionControllers(
                stream=True,
                key="motionControllers",
                left=True,
                right=True,
            ),
            to="bgChildren",
        )

    # Critical diagnostic difference from stock TeleVuer pass-through:
    # explicitly request main-camera updates from the browser. Vuer 0.0.60
    # commonly exports SceneCameraControl rather than OrbitControls.
    if CAMERA_CONTROL_CLASS is None:
        exported = sorted(
            name for name in dir(vuer_schemas)
            if "camera" in name.lower() or "control" in name.lower()
        )
        print(
            "[Probe] ERROR: no supported camera-control schema found. "
            f"Camera/control exports={exported}",
            flush=True,
        )
    else:
        control = None
        errors = []
        for kwargs in (
            {"stream": True, "key": "xr-camera-controls"},
            {"stream": True},
        ):
            try:
                control = CAMERA_CONTROL_CLASS(**kwargs)
                break
            except Exception as exc:
                errors.append(f"{kwargs}: {type(exc).__name__}: {exc}")
        if control is None:
            print(
                f"[Probe] ERROR: found {CAMERA_CONTROL_NAME} but could not construct it: "
                + " | ".join(errors),
                flush=True,
            )
        else:
            session.upsert(control, to="bgChildren")
            print(
                f"[Probe] {CAMERA_CONTROL_NAME}(stream=True) inserted for explicit "
                "camera streaming.",
                flush=True,
            )

    while True:
        await asyncio.sleep(1.0 / self.display_fps)


def pose_info(value: object) -> tuple[np.ndarray | None, str]:
    try:
        pose = np.asarray(value, dtype=float)
    except Exception as exc:
        return None, f"not array-like: {exc}"
    if pose.shape != (4, 4):
        return None, f"shape={pose.shape}, expected (4, 4)"
    if not np.isfinite(pose).all():
        return None, "contains NaN/Inf"
    if float(np.linalg.norm(pose)) < 1e-8:
        return None, "all-zero/uninitialized matrix"
    det = float(np.linalg.det(pose[:3, :3]))
    if abs(det) < 0.5:
        return None, f"singular rotation det={det:.6f}"
    return pose.copy(), f"valid det={det:.6f}"


def yaw_openxr_deg(pose: np.ndarray) -> float:
    forward = -pose[:3, 2].copy()
    forward[1] = 0.0
    norm = float(np.linalg.norm(forward))
    if norm < 1e-8:
        return float("nan")
    forward /= norm
    return math.degrees(math.atan2(float(forward[0]), float(-forward[2])))


def yaw_robot_deg(pose: np.ndarray) -> float:
    forward = pose[:3, 0].copy()
    forward[2] = 0.0
    norm = float(np.linalg.norm(forward))
    if norm < 1e-8:
        return float("nan")
    forward /= norm
    return math.degrees(math.atan2(float(forward[1]), float(forward[0])))


def fmt(value: np.ndarray) -> str:
    return np.array2string(np.asarray(value), precision=3, suppress_small=True, sign=" ")


def print_snapshot(index: int, tv: TeleVuerWrapper) -> None:
    try:
        data = tv.get_tele_data()
    except Exception as exc:
        LOG.exception("[Snapshot %d] get_tele_data failed: %s", index, exc)
        return

    raw, raw_status = pose_info(getattr(tv.tvuer, "head_pose", None))
    processed, processed_status = pose_info(getattr(data, "head_pose", None))
    left, left_status = pose_info(getattr(data, "left_wrist_pose", None))
    right, right_status = pose_info(getattr(data, "right_wrist_pose", None))

    LOG.info("[Snapshot %d] motion_data_ready=%s", index, bool(getattr(data, "motion_data_ready", False)))
    if raw is None:
        LOG.error("[Snapshot %d] RAW head INVALID: %s", index, raw_status)
    else:
        LOG.info(
            "[Snapshot %d] RAW head %s: pos=%s m, yaw(OpenXR)=%+.1f deg",
            index,
            raw_status,
            fmt(raw[:3, 3]),
            yaw_openxr_deg(raw),
        )

    if processed is None:
        LOG.error("[Snapshot %d] PROCESSED head INVALID: %s", index, processed_status)
    else:
        LOG.info(
            "[Snapshot %d] PROCESSED head %s: pos=%s m, yaw(robot)=%+.1f deg",
            index,
            processed_status,
            fmt(processed[:3, 3]),
            yaw_robot_deg(processed),
        )

    if left is None:
        LOG.error("[Snapshot %d] LEFT wrist INVALID: %s", index, left_status)
    else:
        LOG.info("[Snapshot %d] LEFT wrist %s: pos=%s m", index, left_status, fmt(left[:3, 3]))

    if right is None:
        LOG.error("[Snapshot %d] RIGHT wrist INVALID: %s", index, right_status)
    else:
        LOG.info("[Snapshot %d] RIGHT wrist %s: pos=%s m", index, right_status, fmt(right[:3, 3]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-mode", choices=["hand", "controller"], default="hand")
    args = parser.parse_args()

    # Patch before TeleVuerWrapper constructs TeleVuer and forks its server process.
    TeleVuer.on_cam_move = patched_on_cam_move
    TeleVuer.main_pass_through = patched_main_pass_through

    try:
        vuer_version = importlib.metadata.version("vuer")
    except Exception as exc:
        vuer_version = f"unknown ({exc})"
    try:
        televuer_version = importlib.metadata.version("televuer")
    except Exception as exc:
        televuer_version = f"unknown ({exc})"

    LOG.info("[Probe] CAMERA_STREAM_PROBE_V2 -- read-only")
    LOG.info("[Probe] televuer module: %s", Path(televuer.__file__).resolve())
    LOG.info("[Probe] installed televuer=%s, vuer=%s", televuer_version, vuer_version)
    LOG.info("[Probe] Explicit camera streaming and payload logging are enabled.")
    exports = sorted(
        name for name in dir(vuer_schemas)
        if "camera" in name.lower() or "control" in name.lower()
    )
    LOG.info("[Probe] camera/control schema exports: %s", exports)
    if CAMERA_CONTROL_CLASS is None:
        LOG.error("[Probe] No compatible camera-control schema was found.")
    else:
        try:
            signature = inspect.signature(CAMERA_CONTROL_CLASS)
        except Exception as exc:
            signature = f"unavailable ({exc})"
        LOG.info(
            "[Probe] selected camera control: %s, signature=%s",
            CAMERA_CONTROL_NAME,
            signature,
        )

    tv = TeleVuerWrapper(
        use_hand_tracking=args.input_mode == "hand",
        binocular=False,
        img_shape=(480, 640),
        display_mode="pass-through",
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

    LOG.info("[Probe] Enter VR. Turn/walk, then press [p] at several poses. Press [q] to quit.")
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
