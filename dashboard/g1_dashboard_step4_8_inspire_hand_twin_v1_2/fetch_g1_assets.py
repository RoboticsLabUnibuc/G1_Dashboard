#!/usr/bin/env python3
"""Verify/fetch dashboard model assets.

G1 body meshes are copied from local Unitree repositories or fetched from the
official Unitree model repository. Inspire RH56DFX hand meshes are bundled from
the robot-local assets supplied for this dashboard revision and are only
verified here; this helper does not download substitute hand geometry.
"""
from __future__ import annotations

import argparse
import shutil
import sys
import time
import urllib.request
from pathlib import Path

MESH_NAMES = """pelvis pelvis_contour_link left_hip_pitch_link left_hip_roll_link left_hip_yaw_link left_knee_link left_ankle_pitch_link left_ankle_roll_link right_hip_pitch_link right_hip_roll_link right_hip_yaw_link right_knee_link right_ankle_pitch_link right_ankle_roll_link waist_yaw_link waist_roll_link torso_link logo_link head_link waist_support_link left_shoulder_pitch_link left_shoulder_roll_link left_shoulder_yaw_link left_elbow_link left_wrist_roll_link left_wrist_pitch_link left_wrist_yaw_link left_rubber_hand right_shoulder_pitch_link right_shoulder_roll_link right_shoulder_yaw_link right_elbow_link right_wrist_roll_link right_wrist_pitch_link right_wrist_yaw_link right_rubber_hand""".split()

BASE_URL = "https://raw.githubusercontent.com/unitreerobotics/unitree_mujoco/main/unitree_robots/g1/meshes"
HERE = Path(__file__).resolve().parent
DEST = HERE / "static" / "model" / "g1" / "meshes"

INSPIRE_MESH_NAMES = """L_hand_base_link R_hand_base_link Link11_L Link11_R Link12_L Link12_R Link13_L Link13_R Link14_L Link14_R Link15_L Link15_R Link16_L Link16_R Link17_L Link17_R Link18_L Link18_R Link19_L Link19_R Link20_L Link20_R Link21_L Link21_R Link22_L Link22_R""".split()
INSPIRE_DEST = HERE / "static" / "model" / "inspire_hand" / "meshes"

def inspire_status() -> tuple[list[str], list[str]]:
    ok, missing = [], []
    for name in INSPIRE_MESH_NAMES:
        (ok if valid_mesh(INSPIRE_DEST / f"{name}.STL") else missing).append(name)
    return ok, missing


def valid_mesh(path: Path) -> bool:
    try:
        return path.is_file() and path.stat().st_size > 84
    except OSError:
        return False


def default_candidates() -> list[Path]:
    home = Path.home()
    return [
        home / "unitree_mujoco" / "unitree_robots" / "g1" / "meshes",
        home / "unitree_ros" / "robots" / "g1_description" / "meshes",
        home / "xr_teleoperate_g1demo" / "assets" / "g1" / "meshes",
    ]


def copy_from(source: Path, dest: Path) -> int:
    copied = 0
    if not source.is_dir():
        return 0
    for name in MESH_NAMES:
        dst = dest / f"{name}.STL"
        if valid_mesh(dst):
            continue
        src = source / f"{name}.STL"
        if not valid_mesh(src):
            src = source / f"{name}.stl"
        if valid_mesh(src):
            shutil.copy2(src, dst)
            copied += 1
    return copied


def download_one(name: str, dest: Path, retries: int = 3) -> None:
    url = f"{BASE_URL}/{name}.STL"
    target = dest / f"{name}.STL"
    part = target.with_suffix(".STL.part")
    headers = {"User-Agent": "G1-Teleop-Dashboard-Asset-Fetcher/1.0"}
    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            req = urllib.request.Request(url, headers=headers)
            with urllib.request.urlopen(req, timeout=45) as r, part.open("wb") as f:
                shutil.copyfileobj(r, f)
            if not valid_mesh(part):
                raise RuntimeError(f"downloaded file is too small: {part.stat().st_size} bytes")
            part.replace(target)
            return
        except Exception as exc:
            last_exc = exc
            try:
                part.unlink()
            except FileNotFoundError:
                pass
            if attempt < retries:
                time.sleep(attempt)
    raise RuntimeError(f"failed to download {name}: {last_exc}")


def status(dest: Path) -> tuple[list[str], list[str]]:
    ok, missing = [], []
    for name in MESH_NAMES:
        (ok if valid_mesh(dest / f"{name}.STL") else missing).append(name)
    return ok, missing


def main() -> int:
    ap = argparse.ArgumentParser(description="Install official Unitree G1 STL assets for the browser twin")
    ap.add_argument("--source-dir", action="append", default=[], help="local mesh directory to copy from before downloading")
    ap.add_argument("--no-download", action="store_true", help="only search/copy local meshes")
    ap.add_argument("--verify-only", action="store_true", help="do not copy or download; just report status")
    args = ap.parse_args()

    DEST.mkdir(parents=True, exist_ok=True)
    ok, missing = status(DEST)
    if args.verify_only:
        print(f"G1 mesh assets: {len(ok)}/{len(MESH_NAMES)} present")
        iok, imissing = inspire_status()
        print(f"Inspire RH56DFX mesh assets: {len(iok)}/{len(INSPIRE_MESH_NAMES)} present")
        if missing:
            print("Missing G1:", ", ".join(missing))
        if imissing:
            print("Missing Inspire:", ", ".join(imissing))
        return 1 if (missing or imissing) else 0

    sources = [Path(x).expanduser().resolve() for x in args.source_dir] + default_candidates()
    seen: set[Path] = set()
    for source in sources:
        try:
            source = source.resolve()
        except OSError:
            pass
        if source in seen:
            continue
        seen.add(source)
        copied = copy_from(source, DEST)
        if copied:
            print(f"Copied {copied} mesh(es) from {source}")

    ok, missing = status(DEST)
    if missing and not args.no_download:
        print(f"Downloading {len(missing)} missing official Unitree mesh(es)...")
        for i, name in enumerate(missing, 1):
            print(f"[{i:02d}/{len(missing):02d}] {name}.STL", flush=True)
            download_one(name, DEST)

    ok, missing = status(DEST)
    print(f"G1 mesh assets: {len(ok)}/{len(MESH_NAMES)} present in {DEST}")
    iok, imissing = inspire_status()
    print(f"Inspire RH56DFX mesh assets: {len(iok)}/{len(INSPIRE_MESH_NAMES)} present in {INSPIRE_DEST}")
    if missing:
        print("Missing G1:", ", ".join(missing), file=sys.stderr)
    if imissing:
        print("Missing Inspire:", ", ".join(imissing), file=sys.stderr)
    if missing or imissing:
        return 1
    print("Asset install complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
