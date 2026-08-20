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

Allowed display modes:
  rgb, depth, overlay, near, disparity, pointcloud, topdown.

Point-cloud view control is also file-based/local-only:
  - TELEIMAGER_POINT_VIEW_FILE contains validated orbit parameters.
  - yaw/pitch/distance change only the virtual renderer camera, never the D435i.

`disparity`, `pointcloud`, and `topdown` are derived from the same aligned
640x480 RGB + Z16 depth pair. They do not enable additional D435i streams.
"""
from __future__ import annotations

import json
import os
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# PC2 already has the aarch64 pyrealsense2 package under Ubuntu's system
# dist-packages, while Teleimager/aiortc live in the Conda environment. Adding
# the entire system dist-packages directory to PYTHONPATH mixes unrelated
# OpenSSL/cryptography packages and can break aiortc. Expose the system site
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

ALLOWED_MODES = {
    "rgb",
    "depth",
    "overlay",
    "near",
    "disparity",
    "pointcloud",
    "topdown",
}
MODE_FILE = Path(os.environ.get("TELEIMAGER_DISPLAY_MODE_FILE", "/tmp/g1_dashboard_camera_mode.txt"))
STATUS_FILE = Path(os.environ.get("TELEIMAGER_DISPLAY_STATUS_FILE", "/tmp/g1_dashboard_camera_mode_status.json"))
POINT_VIEW_FILE = Path(os.environ.get("TELEIMAGER_POINT_VIEW_FILE", "/tmp/g1_dashboard_point_view.json"))
POINTCLOUD_FILE = Path(os.environ.get("TELEIMAGER_POINTCLOUD_SNAPSHOT_FILE", "/tmp/g1_dashboard_camera_pointcloud.bin"))

POINT_VIEW_DEFAULT = {
    "yaw_deg": 22.0,
    "pitch_deg": 14.0,
    "distance_m": 3.16,
    "target_z_m": 2.0,
}
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

# Derived-view rendering controls. The default 6-pixel sampling step produces about
# 8.6k source points at 640x480, keeping POINT/TOP rendering light enough for
# Jetson while preserving useful scene geometry.
POINT_SAMPLE_STEP = max(2, int(os.environ.get("G1_DASHBOARD_POINT_SAMPLE_STEP", "6")))
POINT_MAX_M = float(os.environ.get("G1_DASHBOARD_POINT_MAX_M", str(DEPTH_FAR_M)))
TOPDOWN_HALF_WIDTH_M = float(os.environ.get("G1_DASHBOARD_TOPDOWN_HALF_WIDTH_M", "2.5"))
TOPDOWN_MAX_FORWARD_M = float(os.environ.get("G1_DASHBOARD_TOPDOWN_MAX_FORWARD_M", str(DEPTH_FAR_M)))
# Camera-relative vertical band (RealSense camera coordinates: +Y is down).
# It removes much of the floor/ceiling from the obstacle plot without claiming
# world-frame height or robot pose knowledge.
TOPDOWN_MIN_Y_M = float(os.environ.get("G1_DASHBOARD_TOPDOWN_MIN_Y_M", "-0.65"))
TOPDOWN_MAX_Y_M = float(os.environ.get("G1_DASHBOARD_TOPDOWN_MAX_Y_M", "0.70"))
TOPDOWN_CELL_M = float(os.environ.get("G1_DASHBOARD_TOPDOWN_CELL_M", "0.10"))
POINTCLOUD_EXPORT_HZ = max(2.0, min(20.0, float(os.environ.get("G1_DASHBOARD_POINTCLOUD_EXPORT_HZ", "15"))))
POINTCLOUD_EXPORT_STEP = max(3, int(os.environ.get("G1_DASHBOARD_POINTCLOUD_EXPORT_STEP", str(POINT_SAMPLE_STEP))))

if not (0.05 <= DEPTH_NEAR_M < DEPTH_FAR_M <= 20.0):
    raise SystemExit("invalid depth visualization range")
if not (DEPTH_NEAR_M <= NEAR_RED_M < NEAR_ORANGE_M < NEAR_YELLOW_M <= DEPTH_FAR_M):
    raise SystemExit("invalid near-field thresholds")
if not (DEPTH_NEAR_M < POINT_MAX_M <= 20.0):
    raise SystemExit("invalid point-cloud range")
if not (0.5 <= TOPDOWN_HALF_WIDTH_M <= 10.0 and DEPTH_NEAR_M < TOPDOWN_MAX_FORWARD_M <= 20.0):
    raise SystemExit("invalid top-down map extent")
if not (TOPDOWN_MIN_Y_M < TOPDOWN_MAX_Y_M):
    raise SystemExit("invalid top-down vertical band")
if not (0.04 <= TOPDOWN_CELL_M <= 0.30):
    raise SystemExit("invalid top-down occupancy cell size")
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


def _sanitize_point_view(raw: object) -> dict[str, float]:
    base = dict(POINT_VIEW_DEFAULT)
    if isinstance(raw, dict):
        for key in base:
            try:
                value = float(raw.get(key, base[key]))
                if np.isfinite(value):
                    base[key] = value
            except Exception:
                pass
    # Deliberately bounded: this is only a virtual renderer camera.
    base["yaw_deg"] = float(np.clip(base["yaw_deg"], -180.0, 180.0))
    base["pitch_deg"] = float(np.clip(base["pitch_deg"], -82.0, 82.0))
    base["distance_m"] = float(np.clip(base["distance_m"], 1.0, 8.0))
    base["target_z_m"] = float(np.clip(base["target_z_m"], 0.5, 5.0))
    return base


def _read_point_view(camera) -> dict[str, float]:
    now = time.monotonic()
    if now < getattr(camera, "_g1_next_point_view_poll", 0.0):
        return getattr(camera, "_g1_point_view", dict(POINT_VIEW_DEFAULT))
    camera._g1_next_point_view_poll = now + 0.06
    view = getattr(camera, "_g1_point_view", dict(POINT_VIEW_DEFAULT))
    try:
        view = _sanitize_point_view(json.loads(POINT_VIEW_FILE.read_text(encoding="utf-8")))
    except FileNotFoundError:
        view = dict(POINT_VIEW_DEFAULT)
    except Exception as exc:
        image_server.logger_mp.debug(f"[G1 camera modes] point-view file read failed: {exc}")
    camera._g1_point_view = view
    return view


def _write_status(camera, mode: str, depth_ready: bool) -> None:
    now_mono = time.monotonic()
    point_view = _read_point_view(camera)
    fingerprint = (
        mode,
        round(point_view["yaw_deg"], 2),
        round(point_view["pitch_deg"], 2),
        round(point_view["distance_m"], 3),
        round(point_view["target_z_m"], 3),
    )
    previous = getattr(camera, "_g1_status_fingerprint", None)
    if fingerprint == previous and now_mono < getattr(camera, "_g1_next_status_write", 0.0):
        return
    camera._g1_next_status_write = now_mono + 0.35
    camera._g1_status_fingerprint = fingerprint
    payload = {
        "schema": "g1_dashboard.camera_display_status.v1.2",
        "mode": mode,
        "depth_ready": bool(depth_ready),
        "width": int(camera._img_shape[1]),
        "height": int(camera._img_shape[0]),
        "fps": float(camera._fps),
        "depth_scale_m_per_unit": float(getattr(camera, "g_depth_scale", 0.0)),
        "derived_modes": ["disparity", "pointcloud", "topdown"],
        "point_view": point_view,
        "topdown_projection": "orthographic_xz_occupancy_grid",
        "topdown_cell_m": float(TOPDOWN_CELL_M),
        "pointcloud_browser_webgl": True,
        "pointcloud_export_hz": float(POINTCLOUD_EXPORT_HZ),
        "pointcloud_left_right_corrected": True,
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


def _depth_products(depth_z16: np.ndarray, depth_scale: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    depth_m = depth_z16.astype(np.float32) * float(depth_scale)
    valid = (depth_z16 > 0) & (depth_m >= DEPTH_NEAR_M) & (depth_m <= DEPTH_FAR_M)
    intensity = np.zeros(depth_z16.shape, dtype=np.uint8)
    if np.any(valid):
        inv = (DEPTH_FAR_M - depth_m[valid]) / (DEPTH_FAR_M - DEPTH_NEAR_M)
        intensity[valid] = np.clip(inv * 255.0, 0.0, 255.0).astype(np.uint8)
    colormap = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
    heat = cv2.applyColorMap(intensity, colormap)
    # Invalid depth is data absence, not a far/near measurement. Dark gray makes
    # the distinction visible without fabricating depth in occlusion holes.
    heat[~valid] = (16, 16, 16)
    return heat, depth_m, valid


def _disparity_view(depth_m: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Render inverse-depth (stereo-disparity-proportional) visualization.

    For a fixed stereo baseline/focal length, disparity is proportional to 1/Z.
    We use that exact relationship for visualization. Absolute disparity pixels
    are unnecessary for the operator view, so no D435i baseline is hard-coded.
    """
    disparity = np.zeros(depth_m.shape, dtype=np.float32)
    disparity[valid] = 1.0 / np.maximum(depth_m[valid], 1e-6)
    near_d = 1.0 / DEPTH_NEAR_M
    far_d = 1.0 / DEPTH_FAR_M
    intensity = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        norm = (disparity[valid] - far_d) / max(near_d - far_d, 1e-6)
        intensity[valid] = np.clip(norm * 255.0, 0.0, 255.0).astype(np.uint8)
    cmap = getattr(cv2, "COLORMAP_MAGMA", getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET))
    output = cv2.applyColorMap(intensity, cmap)
    output[~valid] = (16, 16, 16)
    return output


def _sample_xyz(camera, depth_m: np.ndarray, valid: np.ndarray, step: int) -> tuple[np.ndarray, np.ndarray]:
    """Return camera-frame XYZ points and source pixel coordinates.

    The depth is already aligned to the RGB stream, so the color intrinsics in
    `camera.intrinsics` apply directly to the aligned depth pixels.
    """
    h, w = depth_m.shape
    ys = np.arange(0, h, step, dtype=np.float32)
    xs = np.arange(0, w, step, dtype=np.float32)
    uu, vv = np.meshgrid(xs, ys)
    z = depth_m[::step, ::step]
    good = valid[::step, ::step] & np.isfinite(z) & (z <= POINT_MAX_M)
    if not np.any(good):
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 2), dtype=np.int32)

    intr = camera.intrinsics
    zg = z[good]
    ug = uu[good]
    vg = vv[good]
    x = (ug - float(intr.ppx)) * zg / float(intr.fx)
    y = (vg - float(intr.ppy)) * zg / float(intr.fy)
    points = np.column_stack((x, y, zg)).astype(np.float32, copy=False)
    pixels = np.column_stack((ug.astype(np.int32), vg.astype(np.int32)))
    return points, pixels


def _look_at_basis(eye: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = target - eye
    forward /= max(float(np.linalg.norm(forward)), 1e-6)
    world_up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    # Use a right-handed camera basis. The previous cross-product order made
    # screen-right correspond to -X, mirroring the robot's left/right.
    right = np.cross(world_up, forward)
    right /= max(float(np.linalg.norm(right)), 1e-6)
    up = np.cross(forward, right)
    up /= max(float(np.linalg.norm(up)), 1e-6)
    return right, up, forward


def _publish_pointcloud_snapshot(camera, bgr: np.ndarray, depth_m: np.ndarray, valid: np.ndarray) -> None:
    """Publish a latest-only compact XYZ+RGB packet for browser WebGL.

    Format G1PC v1, little endian:
      32-byte header <4sHHIIdII>
      N packed records <hhhBBB> (millimetres + RGB).

    RealSense coordinates are converted to renderer coordinates (+X right,
    +Y up, +Z forward). The browser owns orbit rendering, so viewpoint changes
    no longer require a round-trip or H.264 frame regeneration.
    """
    now = time.monotonic()
    if now < getattr(camera, "_g1_next_pointcloud_export", 0.0):
        return
    camera._g1_next_pointcloud_export = now + 1.0 / POINTCLOUD_EXPORT_HZ

    points, pixels = _sample_xyz(camera, depth_m, valid, POINTCLOUD_EXPORT_STEP)
    if points.size == 0:
        return
    world = points.copy()
    world[:, 1] *= -1.0  # RealSense +Y down -> WebGL +Y up.
    world[:, 2] *= -1.0  # RealSense +Z forward -> Three.js conventional -Z forward.
    finite = np.all(np.isfinite(world), axis=1)
    world, pixels = world[finite], pixels[finite]
    if world.size == 0:
        return

    xyz_mm = np.rint(world * 1000.0)
    in_range = np.all((xyz_mm >= -32768.0) & (xyz_mm <= 32767.0), axis=1)
    xyz_mm, pixels = xyz_mm[in_range], pixels[in_range]
    if xyz_mm.size == 0:
        return
    xyz_mm = xyz_mm.astype(np.int16, copy=False)
    colors_bgr = bgr[pixels[:, 1], pixels[:, 0]]
    colors_rgb = colors_bgr[:, ::-1].astype(np.uint8, copy=False)

    count = min(int(xyz_mm.shape[0]), 65535)
    xyz_mm = xyz_mm[:count]
    colors_rgb = colors_rgb[:count]
    records = np.empty(
        count,
        dtype=np.dtype([
            ("x", "<i2"), ("y", "<i2"), ("z", "<i2"),
            ("r", "u1"), ("g", "u1"), ("b", "u1"),
        ], align=False),
    )
    records["x"], records["y"], records["z"] = xyz_mm[:, 0], xyz_mm[:, 1], xyz_mm[:, 2]
    records["r"], records["g"], records["b"] = colors_rgb[:, 0], colors_rgb[:, 1], colors_rgb[:, 2]

    seq = (int(getattr(camera, "_g1_pointcloud_seq", 0)) + 1) & 0xFFFFFFFF
    camera._g1_pointcloud_seq = seq
    header = struct.pack(
        "<4sHHIIdII",
        b"G1PC", 1, 9, count, seq, time.time(), 1, 0,
    )
    try:
        POINTCLOUD_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = POINTCLOUD_FILE.with_suffix(POINTCLOUD_FILE.suffix + ".tmp")
        with open(tmp, "wb") as f:
            f.write(header)
            f.write(records.tobytes(order="C"))
        os.chmod(tmp, 0o600)
        tmp.replace(POINTCLOUD_FILE)
    except Exception as exc:
        image_server.logger_mp.debug(f"[G1 camera modes] point-cloud snapshot write failed: {exc}")


def _pointcloud_view(camera, bgr: np.ndarray, depth_m: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Render an RGB-textured point cloud from an orbitable virtual viewpoint."""
    h, w = depth_m.shape
    canvas = np.full((h, w, 3), (8, 12, 17), dtype=np.uint8)
    points, pixels = _sample_xyz(camera, depth_m, valid, POINT_SAMPLE_STEP)
    if points.size == 0:
        cv2.putText(canvas, "POINT CLOUD - NO VALID DEPTH", (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 220, 230), 1, cv2.LINE_AA)
        return canvas

    world = points.copy()
    world[:, 1] *= -1.0  # RealSense +Y down -> renderer +Y up.

    view = _read_point_view(camera)
    yaw = np.deg2rad(view["yaw_deg"])
    pitch = np.deg2rad(view["pitch_deg"])
    radius = view["distance_m"]
    target = np.array([0.0, 0.0, view["target_z_m"]], dtype=np.float32)
    cp = float(np.cos(pitch))
    eye = target + np.array([
        radius * cp * np.sin(yaw),
        radius * np.sin(pitch),
        -radius * cp * np.cos(yaw),
    ], dtype=np.float32)

    right, up, forward = _look_at_basis(eye, target)
    rel = world - eye
    qx = rel @ right
    qy = rel @ up
    qz = rel @ forward
    keep = qz > 0.12
    if not np.any(keep):
        return canvas

    qx, qy, qz = qx[keep], qy[keep], qz[keep]
    pix = pixels[keep]
    focal = float(min(w, h)) * 0.92
    su = np.rint((w * 0.50) + focal * (qx / qz)).astype(np.int32)
    sv = np.rint((h * 0.52) - focal * (qy / qz)).astype(np.int32)
    inside = (su >= 1) & (su < w - 1) & (sv >= 1) & (sv < h - 1)
    if not np.any(inside):
        return canvas

    su, sv, qz, pix = su[inside], sv[inside], qz[inside], pix[inside]
    colors = bgr[pix[:, 1], pix[:, 0]]

    order = np.argsort(qz)[::-1]
    su, sv, colors = su[order], sv[order], colors[order]
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
        canvas[sv + dy, su + dx] = colors

    cv2.putText(canvas, "POINT CLOUD | DRAG TO ORBIT / WHEEL TO ZOOM", (16, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (205, 220, 235), 1, cv2.LINE_AA)
    cv2.putText(
        canvas,
        f"yaw {view['yaw_deg']:+.0f}  pitch {view['pitch_deg']:+.0f}  distance {view['distance_m']:.1f}m",
        (16, h - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (145, 165, 185),
        1,
        cv2.LINE_AA,
    )
    return canvas

def _topdown_view(camera, depth_m: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Orthographically project 3-D returns into a binned X/Z occupancy grid.

    This is a true 2-D projection: Y is discarded after a camera-relative
    vertical obstacle filter. Unlike the previous raw-point splat, points are
    aggregated into metric grid cells so the result reads like a map rather
    than a rotated point cloud.
    """
    h, w = depth_m.shape
    canvas = np.full((h, w, 3), (9, 14, 19), dtype=np.uint8)
    map_left, map_right = 34, w - 22
    map_top, map_bottom = 34, h - 30
    center_x = (map_left + map_right) // 2

    grid_w = max(1, int(np.ceil((2.0 * TOPDOWN_HALF_WIDTH_M) / TOPDOWN_CELL_M)))
    grid_h = max(1, int(np.ceil(TOPDOWN_MAX_FORWARD_M / TOPDOWN_CELL_M)))
    counts = np.zeros((grid_h, grid_w), dtype=np.uint16)

    points, _ = _sample_xyz(camera, depth_m, valid, POINT_SAMPLE_STEP)
    if points.size:
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        keep = (
            (z >= DEPTH_NEAR_M)
            & (z <= TOPDOWN_MAX_FORWARD_M)
            & (np.abs(x) <= TOPDOWN_HALF_WIDTH_M)
            & (y >= TOPDOWN_MIN_Y_M)
            & (y <= TOPDOWN_MAX_Y_M)
        )
        x, z = x[keep], z[keep]
        if x.size:
            ix = np.floor((x + TOPDOWN_HALF_WIDTH_M) / TOPDOWN_CELL_M).astype(np.int32)
            iz = np.floor(z / TOPDOWN_CELL_M).astype(np.int32)
            inside = (ix >= 0) & (ix < grid_w) & (iz >= 0) & (iz < grid_h)
            np.add.at(counts, (iz[inside], ix[inside]), 1)

    # Filled metric occupancy cells. Requiring two decimated source samples
    # suppresses most isolated stereo speckles while preserving real surfaces.
    occupied = counts >= 2
    cell_px_w = (map_right - map_left) / float(grid_w)
    cell_px_h = (map_bottom - map_top) / float(grid_h)
    occ_z, occ_x = np.nonzero(occupied)
    for iz, ix in zip(occ_z.tolist(), occ_x.tolist()):
        z_center = (iz + 0.5) * TOPDOWN_CELL_M
        if z_center < NEAR_RED_M:
            color = (0, 0, 255)
        elif z_center < NEAR_ORANGE_M:
            color = (0, 150, 255)
        elif z_center < NEAR_YELLOW_M:
            color = (0, 235, 235)
        else:
            color = (180, 160, 50)
        x0 = int(round(map_left + ix * cell_px_w))
        x1 = int(round(map_left + (ix + 1) * cell_px_w))
        y1 = int(round(map_bottom - iz * cell_px_h))
        y0 = int(round(map_bottom - (iz + 1) * cell_px_h))
        cv2.rectangle(canvas, (x0, y0), (max(x0, x1 - 1), max(y0, y1 - 1)), color, -1)

    # Metric grid drawn after occupancy so scale remains legible.
    cv2.rectangle(canvas, (map_left, map_top), (map_right, map_bottom), (64, 82, 96), 1)
    for metres in np.arange(0.5, TOPDOWN_MAX_FORWARD_M + 0.001, 0.5):
        py = int(round(map_bottom - (metres / TOPDOWN_MAX_FORWARD_M) * (map_bottom - map_top)))
        major = abs(metres - round(metres)) < 1e-4
        shade = (71, 89, 103) if major else (39, 51, 61)
        cv2.line(canvas, (map_left, py), (map_right, py), shade, 1)
        if major:
            cv2.putText(canvas, f"{metres:.0f}m", (map_left + 4, max(map_top + 12, py - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 166, 180), 1, cv2.LINE_AA)
    for lateral in np.arange(-2.0, 2.01, 1.0):
        if abs(lateral) > TOPDOWN_HALF_WIDTH_M:
            continue
        px = int(round(center_x + (lateral / TOPDOWN_HALF_WIDTH_M) * ((map_right - map_left) / 2.0)))
        cv2.line(canvas, (px, map_top), (px, map_bottom), (45, 59, 70), 1)

    cv2.line(canvas, (center_x, map_bottom), (center_x, map_top), (64, 87, 103), 1)
    tri = np.array([[center_x, map_bottom - 10], [center_x - 7, map_bottom], [center_x + 7, map_bottom]], dtype=np.int32)
    cv2.fillConvexPoly(canvas, tri, (220, 232, 240))

    cv2.putText(canvas, "2-D X/Z OCCUPANCY", (map_left, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (210, 225, 235), 1, cv2.LINE_AA)
    cv2.putText(canvas, "ORTHOGRAPHIC / CAMERA-RELATIVE", (w - 275, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (130, 151, 169), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"cell={TOPDOWN_CELL_M:.2f}m  Y={TOPDOWN_MIN_Y_M:+.2f}..{TOPDOWN_MAX_Y_M:+.2f}m", (map_left, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (110, 132, 150), 1, cv2.LINE_AA)
    return canvas

def _render_mode(camera, bgr: np.ndarray, depth_z16: np.ndarray | None, depth_scale: float, mode: str) -> np.ndarray:
    if mode == "rgb" or depth_z16 is None:
        return bgr

    heat, depth_m, valid = _depth_products(depth_z16, depth_scale)

    if mode == "depth":
        return heat

    if mode == "overlay":
        mixed = cv2.addWeighted(bgr, 0.58, heat, 0.42, 0.0)
        # Preserve real RGB where depth is invalid; do not paint stereo holes as
        # fake black geometry.
        mixed[~valid] = bgr[~valid]
        return mixed

    if mode == "near":
        # Operator-oriented near-field visualization. Keep the scene
        # recognizable, then strongly mark only valid pixels inside bands.
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

    if mode == "disparity":
        return _disparity_view(depth_m, valid)

    if mode == "pointcloud":
        return _pointcloud_view(camera, bgr, depth_m, valid)

    if mode == "topdown":
        return _topdown_view(camera, depth_m, valid)

    return bgr


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
    self._g1_status_fingerprint = None
    self._g1_point_view = dict(POINT_VIEW_DEFAULT)
    self._g1_next_point_view_poll = 0.0
    self._g1_next_pointcloud_export = 0.0
    self._g1_pointcloud_seq = 0


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
    depth_scale = getattr(self, "g_depth_scale", 0.001)
    if mode == "pointcloud" and depth_numpy is not None:
        _heat, depth_m, valid = _depth_products(depth_numpy, depth_scale)
        _publish_pointcloud_snapshot(self, bgr_numpy, depth_m, valid)
    output = _render_mode(self, bgr_numpy, depth_numpy, depth_scale, mode)

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
    "[G1 camera modes] wrapper enabled: config=%s mode_file=%s point_view_file=%s pointcloud_file=%s status_file=%s modes=%s",
    CONFIG_FILE,
    MODE_FILE,
    POINT_VIEW_FILE,
    POINTCLOUD_FILE,
    STATUS_FILE,
    ",".join(sorted(ALLOWED_MODES)),
)

if __name__ == "__main__":
    image_server.main()
