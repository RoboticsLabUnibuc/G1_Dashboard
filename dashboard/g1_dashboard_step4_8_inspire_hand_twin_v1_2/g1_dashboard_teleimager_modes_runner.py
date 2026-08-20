#!/usr/bin/env python3
"""Dashboard-owned Teleimager launcher with RealSense display-mode rendering.

This wrapper deliberately leaves the upstream Teleimager checkout untouched.
It imports Teleimager from the validated `teleimager` conda environment, forces
full-resolution depth on the configured head RealSense camera, and changes only
which already-aligned BGR frame is written to the existing Teleimager WebRTC
buffer. The WebRTC server/port and client sessions stay alive across mode
changes.

Control is file-based and local-only:
  - TELEIMAGER_DISPLAY_MODE_FILE contains one exact mode token.
  - TELEIMAGER_DISPLAY_STATUS_FILE receives the last mode rendered.

Allowed display modes: rgb, depth, overlay, near.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# PC2 already has the aarch64 pyrealsense2 package under Ubuntu's system
# dist-packages, while Teleimager/aiortc live in the Conda environment.  Adding
# the entire system dist-packages directory to PYTHONPATH mixes unrelated
# OpenSSL/cryptography packages and can break aiortc.  Expose the system site
# only for this one import, then restore the Conda-first import path.
_RS_SYSTEM_SITE = os.environ.get(
    "G1_DASHBOARD_REALSENSE_SYSTEM_SITE",
    "/usr/lib/python3/dist-packages",
)
if _RS_SYSTEM_SITE not in sys.path:
    sys.path.insert(0, _RS_SYSTEM_SITE)
    try:
        import pyrealsense2 as _g1_pyrealsense2  # noqa: F401
    finally:
        try:
            sys.path.remove(_RS_SYSTEM_SITE)
        except ValueError:
            pass
else:
    import pyrealsense2 as _g1_pyrealsense2  # noqa: F401

import teleimager.image_server as image_server

ALLOWED_MODES = {"rgb", "depth", "overlay", "near"}
MODE_FILE = Path(os.environ.get("TELEIMAGER_DISPLAY_MODE_FILE", "/tmp/g1_dashboard_camera_mode.txt"))
STATUS_FILE = Path(os.environ.get("TELEIMAGER_DISPLAY_STATUS_FILE", "/tmp/g1_dashboard_camera_mode_status.json"))
CONFIG_FILE = Path(
    os.environ.get(
        "G1_DASHBOARD_CAMERA_CONFIG",
        str(Path(__file__).with_name("cam_config_realsense_modes.yaml")),
    )
).expanduser()
DEPTH_NEAR_M = float(os.environ.get("G1_DASHBOARD_DEPTH_NEAR_M", "0.25"))
DEPTH_FAR_M = float(os.environ.get("G1_DASHBOARD_DEPTH_FAR_M", "4.0"))
NEAR_RED_M = float(os.environ.get("G1_DASHBOARD_NEAR_RED_M", "0.50"))
NEAR_ORANGE_M = float(os.environ.get("G1_DASHBOARD_NEAR_ORANGE_M", "1.00"))
NEAR_YELLOW_M = float(os.environ.get("G1_DASHBOARD_NEAR_YELLOW_M", "2.00"))

if not (0.05 <= DEPTH_NEAR_M < DEPTH_FAR_M <= 20.0):
    raise SystemExit("invalid depth visualization range")
if not (DEPTH_NEAR_M <= NEAR_RED_M < NEAR_ORANGE_M < NEAR_YELLOW_M <= DEPTH_FAR_M):
    raise SystemExit("invalid near-field thresholds")
if not CONFIG_FILE.is_file():
    raise SystemExit(f"dashboard RealSense camera config not found: {CONFIG_FILE}")


def _read_requested_mode(camera) -> str:
    now = time.monotonic()
    if now < getattr(camera, "_g1_next_mode_poll", 0.0):
        return getattr(camera, "_g1_display_mode", "rgb")
    camera._g1_next_mode_poll = now + 0.10
    mode = "rgb"
    try:
        raw = MODE_FILE.read_text(encoding="utf-8").strip().lower()
        if raw in ALLOWED_MODES:
            mode = raw
    except FileNotFoundError:
        pass
    except Exception as exc:
        image_server.logger_mp.warning(f"[G1 camera modes] mode file read failed: {exc}")
        mode = getattr(camera, "_g1_display_mode", "rgb")
    camera._g1_display_mode = mode
    return mode


def _write_status(camera, mode: str, depth_ready: bool) -> None:
    now_mono = time.monotonic()
    previous = getattr(camera, "_g1_status_mode", None)
    if mode == previous and now_mono < getattr(camera, "_g1_next_status_write", 0.0):
        return
    camera._g1_next_status_write = now_mono + 0.50
    camera._g1_status_mode = mode
    payload = {
        "schema": "g1_dashboard.camera_display_status.v1",
        "mode": mode,
        "depth_ready": bool(depth_ready),
        "width": int(camera._img_shape[1]),
        "height": int(camera._img_shape[0]),
        "fps": float(camera._fps),
        "depth_scale_m_per_unit": float(getattr(camera, "g_depth_scale", 0.0)),
        "updated_unix_time_s": time.time(),
    }
    try:
        STATUS_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATUS_FILE.with_suffix(STATUS_FILE.suffix + ".tmp")
        tmp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
        os.chmod(tmp, 0o600)
        tmp.replace(STATUS_FILE)
    except Exception as exc:
        image_server.logger_mp.debug(f"[G1 camera modes] status write failed: {exc}")


def _depth_heatmap(depth_z16: np.ndarray, depth_scale: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    depth_m = depth_z16.astype(np.float32) * float(depth_scale)
    valid = (depth_z16 > 0) & (depth_m >= DEPTH_NEAR_M) & (depth_m <= DEPTH_FAR_M)
    intensity = np.zeros(depth_z16.shape, dtype=np.uint8)
    if np.any(valid):
        inv = (DEPTH_FAR_M - depth_m[valid]) / (DEPTH_FAR_M - DEPTH_NEAR_M)
        intensity[valid] = np.clip(inv * 255.0, 0.0, 255.0).astype(np.uint8)
    colormap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    heat = cv2.applyColorMap(intensity, colormap)
    heat[~valid] = 0
    return heat, depth_m, valid


def _render_mode(bgr: np.ndarray, depth_z16: np.ndarray | None, depth_scale: float, mode: str) -> np.ndarray:
    if mode == "rgb" or depth_z16 is None:
        return bgr

    heat, depth_m, valid = _depth_heatmap(depth_z16, depth_scale)

    if mode == "depth":
        return heat

    if mode == "overlay":
        mixed = cv2.addWeighted(bgr, 0.58, heat, 0.42, 0.0)
        mixed[~valid] = bgr[~valid]
        return mixed

    # Operator-oriented near-field visualization. Keep the scene recognizable,
    # then strongly mark only valid pixels inside the configured distance bands.
    output = cv2.convertScaleAbs(bgr, alpha=0.38, beta=0)
    tint = np.zeros_like(bgr)
    red = valid & (depth_m < NEAR_RED_M)
    orange = valid & (depth_m >= NEAR_RED_M) & (depth_m < NEAR_ORANGE_M)
    yellow = valid & (depth_m >= NEAR_ORANGE_M) & (depth_m < NEAR_YELLOW_M)
    tint[red] = (0, 0, 255)
    tint[orange] = (0, 165, 255)
    tint[yellow] = (0, 255, 255)
    marked = red | orange | yellow
    if np.any(marked):
        blended = cv2.addWeighted(bgr, 0.28, tint, 0.72, 0.0)
        output[marked] = blended[marked]
    return output


_original_rs_init = image_server.RealSenseCamera.__init__


def _patched_rs_init(
    self,
    cam_topic,
    serial_number,
    img_shape,
    fps,
    enable_zmq=True,
    zmq_port=55555,
    enable_webrtc=False,
    webrtc_port=66666,
    webrtc_codec=None,
    enable_depth=False,
):
    # The dashboard's head-camera config is RealSense-only. Force depth for the
    # head camera while retaining the upstream constructor and acquisition path.
    _original_rs_init(
        self,
        cam_topic,
        serial_number,
        img_shape,
        fps,
        enable_zmq,
        zmq_port,
        enable_webrtc,
        webrtc_port,
        webrtc_codec,
        enable_depth=(True if str(cam_topic) == "head_camera" else enable_depth),
    )
    self._g1_display_mode = "rgb"
    self._g1_next_mode_poll = 0.0
    self._g1_next_status_write = 0.0
    self._g1_status_mode = None


def _patched_rs_update_frame(self):
    frames = self.pipeline.wait_for_frames()
    aligned_frames = self.align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    if not color_frame:
        return None

    depth_numpy = None
    if self._enable_depth:
        depth_frame = aligned_frames.get_depth_frame()
        if depth_frame:
            depth_numpy = np.asanyarray(depth_frame.get_data())
            self._latest_depth = depth_numpy
        else:
            self._latest_depth = None

    bgr_numpy = np.asanyarray(color_frame.get_data())
    mode = _read_requested_mode(self)
    output = _render_mode(bgr_numpy, depth_numpy, getattr(self, "g_depth_scale", 0.001), mode)

    if self._enable_webrtc:
        self._webrtc_buffer.write(output)

    if self._enable_zmq:
        ok, buf = cv2.imencode(".jpg", output)
        if ok:
            self._zmq_buffer.write(buf.tobytes())

    _write_status(self, mode, depth_numpy is not None)
    if not self._ready.is_set():
        self._ready.set()


image_server.RealSenseCamera.__init__ = _patched_rs_init
image_server.RealSenseCamera._update_frame = _patched_rs_update_frame
image_server.CONFIG_PATH = str(CONFIG_FILE)

# The official Teleimager CLI requires --rs to allow RealSense construction.
# Preserve optional --no-affinity if the dashboard launch ever supplies it.
if "--rs" not in sys.argv:
    sys.argv.append("--rs")

image_server.logger_mp.info(
    "[G1 camera modes] wrapper enabled: config=%s mode_file=%s status_file=%s modes=%s",
    CONFIG_FILE,
    MODE_FILE,
    STATUS_FILE,
    ",".join(sorted(ALLOWED_MODES)),
)

if __name__ == "__main__":
    image_server.main()
