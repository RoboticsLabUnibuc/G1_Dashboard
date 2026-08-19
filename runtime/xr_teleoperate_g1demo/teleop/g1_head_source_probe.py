#!/usr/bin/env python3
"""Read-only probe that compares raw Vuer head tracking with processed TeleVuer data.

No DDS, IK, simulator, or robot commands are created.

Keys:
  r  capture baseline
  p  print one numbered snapshot
  q  quit
"""

from __future__ import annotations

import argparse
import inspect
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening
import televuer
from televuer import TeleVuerWrapper

STOP = False
CAPTURE = False
SNAPSHOT = False


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_head_source_probe")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)-8s %(message)s", "%H:%M:%S"))
    logger.addHandler(handler)
    logger.propagate = False
    return logger


LOG = configure_logging()


def on_press(key: str) -> None:
    global STOP, CAPTURE, SNAPSHOT
    if key == "r":
        CAPTURE = True
    elif key == "p":
        SNAPSHOT = True
    elif key == "q":
        STOP = True
        try:
            stop_listening()
        except Exception:
            pass
    else:
        LOG.warning("Key %r has no action. Use r, p, or q.", key)


def as_pose(value: object) -> np.ndarray | None:
    try:
        pose = np.asarray(value, dtype=float)
    except Exception:
        return None
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        return None
    if np.linalg.norm(pose) < 1e-8:
        return None
    if abs(float(np.linalg.det(pose[:3, :3]))) < 0.5:
        return None
    return pose.copy()


def rotation_delta_deg(reference: np.ndarray, current: np.ndarray) -> float:
    rel = reference[:3, :3].T @ current[:3, :3]
    c = float(np.clip((np.trace(rel) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(c))


def yaw_robot_deg(pose: np.ndarray) -> float:
    # Valid for a robot-basis world-from-head pose: x=forward, y=left, z=up.
    x_axis = pose[:3, 0].copy()
    x_axis[2] = 0.0
    n = float(np.linalg.norm(x_axis))
    if n < 1e-8:
        return float("nan")
    x_axis /= n
    return math.degrees(math.atan2(float(x_axis[1]), float(x_axis[0])))


def fmt(v: np.ndarray) -> str:
    return np.array2string(np.asarray(v), precision=3, suppress_small=True, sign=" ")


T_ROBOT_OPENXR = np.array(
    [[0.0, 0.0, -1.0, 0.0],
     [-1.0, 0.0, 0.0, 0.0],
     [0.0, 1.0, 0.0, 0.0],
     [0.0, 0.0, 0.0, 1.0]],
    dtype=float,
)
T_OPENXR_ROBOT = np.linalg.inv(T_ROBOT_OPENXR)


def raw_head_in_robot_basis(raw_head: np.ndarray) -> np.ndarray:
    return T_ROBOT_OPENXR @ raw_head @ T_OPENXR_ROBOT


@dataclass
class Baseline:
    raw_head: np.ndarray
    processed_head: np.ndarray
    left_wrist: np.ndarray
    right_wrist: np.ndarray


def get_all(tv: TeleVuerWrapper):
    data = tv.get_tele_data()
    raw_head = as_pose(getattr(tv.tvuer, "head_pose", None))
    processed_head = as_pose(getattr(data, "head_pose", None))
    left = as_pose(getattr(data, "left_wrist_pose", None))
    right = as_pose(getattr(data, "right_wrist_pose", None))
    return data, raw_head, processed_head, left, right


def capture_stable(tv: TeleVuerWrapper, frames: int = 20) -> Baseline | None:
    samples = []
    last_warn = 0.0
    LOG.info("[Probe] Hold head and arms still while baseline is captured...")
    while not STOP:
        _, raw, processed, left, right = get_all(tv)
        if raw is None or processed is None or left is None or right is None:
            samples.clear()
            now = time.monotonic()
            if now - last_warn >= 1.0:
                LOG.warning("[Probe] Waiting for valid raw head, processed head, and both wrists.")
                last_warn = now
            time.sleep(0.02)
            continue
        if samples:
            prev = samples[-1]
            max_step = max(
                np.linalg.norm(raw[:3, 3] - prev[0][:3, 3]),
                np.linalg.norm(processed[:3, 3] - prev[1][:3, 3]),
                np.linalg.norm(left[:3, 3] - prev[2][:3, 3]),
                np.linalg.norm(right[:3, 3] - prev[3][:3, 3]),
            )
            max_rot = max(
                rotation_delta_deg(prev[0], raw),
                rotation_delta_deg(prev[1], processed),
                rotation_delta_deg(prev[2], left),
                rotation_delta_deg(prev[3], right),
            )
            if max_step > 0.025 or max_rot > 5.0:
                samples.clear()
        samples.append((raw, processed, left, right))
        if len(samples) >= frames:
            raw, processed, left, right = samples[-1]
            return Baseline(raw, processed, left, right)
        time.sleep(0.02)
    return None


def log_snapshot(index: int, baseline: Baseline, tv: TeleVuerWrapper) -> None:
    _, raw, processed, left, right = get_all(tv)
    if raw is None or processed is None or left is None or right is None:
        LOG.warning("[Snapshot %d] Invalid tracking sample.", index)
        return

    raw_robot = raw_head_in_robot_basis(raw)
    raw_base_robot = raw_head_in_robot_basis(baseline.raw_head)

    raw_dt = raw[:3, 3] - baseline.raw_head[:3, 3]
    proc_dt = processed[:3, 3] - baseline.processed_head[:3, 3]
    left_dt = left[:3, 3] - baseline.left_wrist[:3, 3]
    right_dt = right[:3, 3] - baseline.right_wrist[:3, 3]
    mean_dt = 0.5 * (left_dt + right_dt)
    sep0 = baseline.left_wrist[:3, 3] - baseline.right_wrist[:3, 3]
    sep1 = left[:3, 3] - right[:3, 3]

    raw_yaw_delta = math.degrees(
        math.atan2(
            math.sin(math.radians(yaw_robot_deg(raw_robot) - yaw_robot_deg(raw_base_robot))),
            math.cos(math.radians(yaw_robot_deg(raw_robot) - yaw_robot_deg(raw_base_robot))),
        )
    )
    proc_yaw_delta = math.degrees(
        math.atan2(
            math.sin(math.radians(yaw_robot_deg(processed) - yaw_robot_deg(baseline.processed_head))),
            math.cos(math.radians(yaw_robot_deg(processed) - yaw_robot_deg(baseline.processed_head))),
        )
    )

    LOG.info(
        "[Snapshot %d] RAW head: dpos=%s m (norm %.3f), dRot=%.1f deg, dYaw(robot basis)=%+.1f deg",
        index, fmt(raw_dt), float(np.linalg.norm(raw_dt)),
        rotation_delta_deg(baseline.raw_head, raw), raw_yaw_delta,
    )
    LOG.info(
        "[Snapshot %d] PROCESSED head: dpos=%s m (norm %.3f), dRot=%.1f deg, dYaw=%+.1f deg",
        index, fmt(proc_dt), float(np.linalg.norm(proc_dt)),
        rotation_delta_deg(baseline.processed_head, processed), proc_yaw_delta,
    )
    LOG.info(
        "[Snapshot %d] PROCESSED wrists: mean d=%s m, L d=%s m, R d=%s m, separation d=%s m, L/R dRot=%.1f/%.1f deg",
        index, fmt(mean_dt), fmt(left_dt), fmt(right_dt), fmt(sep1 - sep0),
        rotation_delta_deg(baseline.left_wrist, left),
        rotation_delta_deg(baseline.right_wrist, right),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-mode", choices=["hand", "controller"], default="hand")
    parser.add_argument("--display-mode", choices=["immersive", "ego", "pass-through"], default="pass-through")
    args = parser.parse_args()

    LOG.info("[Probe] HEAD_SOURCE_PROBE_V1 -- read-only")
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

    LOG.info("[Probe] Enter VR. Press [r] once at 0 deg. Press [p] at 0, 45, 90, walk point 1, walk point 2. Press [q] to quit.")
    baseline: Baseline | None = None
    snapshot_index = 0
    global CAPTURE, SNAPSHOT

    try:
        while not STOP:
            if CAPTURE:
                CAPTURE = False
                baseline = capture_stable(tv)
                if baseline is not None:
                    LOG.info("[Probe] Baseline captured.")
            if SNAPSHOT:
                SNAPSHOT = False
                if baseline is None:
                    LOG.warning("[Probe] Press [r] before [p].")
                else:
                    snapshot_index += 1
                    log_snapshot(snapshot_index, baseline, tv)
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
