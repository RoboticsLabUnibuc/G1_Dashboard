#!/usr/bin/env python3
"""Isolated ROS 2 SLAM snapshot and initial-pose worker."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
import sys
import time
from array import array
from pathlib import Path
from typing import Any

sys.path.append("/opt/ros/humble/lib/python3.10/site-packages")

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String as RosString
from unitree_api.msg import Request as UnitreeRequest
from unitree_api.msg import Response as UnitreeResponse


SCHEMA = "g1_dashboard.slam.v1"
DEFAULT_MAP = Path.home() / "g1_ws/map/harta_buna_2707.pcd"
DEFAULT_NATIVE_MAP = Path(
    "/home/unitree/.slam_save_harta_buna_2707_1785162889674.pcd"
)
DEFAULT_STATE_DIR = Path(
    os.environ.get(
        "G1_DASHBOARD_SLAM_STATE_DIR",
        f"/tmp/g1_dashboard_slam_{os.getuid()}",
    )
)
DEFAULT_MAX_POINTS = 12000
POSE_MAX_AGE_S = 1.5
CLOUD_MAX_AGE_S = 1.5
INITIALIZE_API_ID = 1804
INITIALIZE_TIMEOUT_S = 20.0
INITIALIZE_SCHEMA = "g1_dashboard.slam.initialize.v1"


def finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def quaternion_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def pose_from_mapping(value: Any) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None

    x = finite(value.get("x"))
    y = finite(value.get("y"))
    if x is None or y is None:
        return None

    yaw = finite(value.get("yaw"))
    if yaw is None:
        qx = finite(value.get("q_x"))
        qy = finite(value.get("q_y"))
        qz = finite(value.get("q_z"))
        qw = finite(value.get("q_w"))
        if None in (qx, qy, qz, qw):
            return None
        yaw = quaternion_yaw(qx, qy, qz, qw)

    return {"x": x, "y": y, "yaw": yaw}


def pose_from_odometry(message: Odometry) -> dict[str, float]:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return {
        "x": float(position.x),
        "y": float(position.y),
        "yaw": quaternion_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
    }


def atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, separators=(",", ":"), sort_keys=True),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def atomic_bytes(path: Path, value: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_bytes(value)
    os.replace(temporary, path)


def inspect_pcd(path: Path) -> dict[str, Any]:
    result = {
        "path": str(path),
        "available": False,
        "bytes": None,
        "points": None,
        "fields": [],
        "data": None,
    }

    try:
        result["bytes"] = path.stat().st_size
        with path.open("rb") as handle:
            header = handle.read(8192).decode("ascii", errors="replace")
    except OSError as exc:
        result["error"] = str(exc)
        return result

    for line in header.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = stripped.split()
        key = parts[0].upper()
        if key == "FIELDS":
            result["fields"] = parts[1:]
        elif key == "POINTS" and len(parts) >= 2:
            try:
                result["points"] = int(parts[1])
            except ValueError:
                pass
        elif key == "DATA" and len(parts) >= 2:
            result["data"] = parts[1].lower()
            break

    result["available"] = bool(
        result["points"]
        and all(name in result["fields"] for name in ("x", "y", "z"))
        and result["data"] in {"ascii", "binary"}
    )
    return result


class SlamSnapshotWorker(Node):
    def __init__(
        self,
        *,
        map_path: Path,
        native_map_path: Path,
        state_dir: Path,
        maximum_points: int,
    ) -> None:
        super().__init__("g1_dashboard_slam_snapshot")

        self.map_path = map_path
        self.native_map_path = native_map_path
        self.native_map_address: str | None = None
        self.native_pcd_name: str | None = None
        self.native_map_observed = False
        self.native_map_matches = False
        self.state_dir = state_dir
        self.status_path = state_dir / "status.json"
        self.cloud_path = state_dir / "live_cloud_f32.bin"
        self.initialize_path = state_dir / "initialize_request.json"
        self.maximum_points = max(500, min(50000, int(maximum_points)))

        try:
            self.initialize_last_mtime_ns = (
                self.initialize_path.stat().st_mtime_ns
            )
        except OSError:
            self.initialize_last_mtime_ns = 0

        self.initialization: dict[str, Any] = {
            "state": "IDLE",
            "request_id": None,
            "requested_pose": None,
            "response": None,
            "error": None,
        }
        self.initialization_started_monotonic = 0.0

        self.pose_sources: dict[str, tuple[float, dict[str, float]]] = {}
        self.cloud_sequence = 0
        self.cloud_received_monotonic = 0.0
        self.cloud_source: str | None = None
        self.cloud_points = 0
        self.cloud_error: str | None = None
        self.localized_seen = False
        self.started_monotonic = time.monotonic()
        self.map_info = inspect_pcd(map_path)

        request_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        response_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.slam_request_publisher = self.create_publisher(
            UnitreeRequest,
            "/api/slam_operate/request",
            request_qos,
        )
        self.create_subscription(
            UnitreeResponse,
            "/api/slam_operate/response",
            self.slam_response_callback,
            response_qos,
        )

        self.create_subscription(
            RosString,
            "/slam_info",
            self.slam_info_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/unitree/slam_relocation/odom",
            lambda message: self.odom_callback(message, "relocation_odom"),
            10,
        )
        self.create_subscription(
            Odometry,
            "/state_estimator/fusion_odom",
            lambda message: self.odom_callback(message, "fusion_odom"),
            10,
        )
        self.create_subscription(
            Odometry,
            "/state_estimator/odom_pelvis",
            lambda message: self.odom_callback(message, "odom_pelvis"),
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/unitree/slam_relocation/points",
            self.relocation_cloud_callback,
            10,
        )

        self.create_timer(0.1, self.poll_initialize_request)
        self.create_timer(0.1, self.publish_status)
        self.publish_status()

        self.get_logger().info(
            "SLAM snapshot worker ready: "
            f"map={map_path} state_dir={state_dir} "
            f"maximum_points={self.maximum_points}"
        )

    def odom_callback(self, message: Odometry, source: str) -> None:
        self.pose_sources[source] = (
            time.monotonic(),
            pose_from_odometry(message),
        )

    def slam_info_callback(self, message: RosString) -> None:
        try:
            root = json.loads(message.data)
            if str(root.get("type", "")) != "pos_info":
                return
            data = root.get("data") or {}
            address = str(data.get("address") or "").strip()
            pcd_name = str(data.get("pcdName") or "").strip()

            self.native_map_address = address or None
            self.native_pcd_name = pcd_name or None
            self.native_map_observed = bool(address or pcd_name)

            address_match = bool(
                address
                and os.path.normpath(address)
                    == os.path.normpath(str(self.native_map_path))
            )
            expected_names = {
                self.map_path.name,
                self.map_path.stem,
            }
            observed_names = (
                {
                    Path(pcd_name).name,
                    Path(pcd_name).stem,
                }
                if pcd_name
                else set()
            )
            self.native_map_matches = bool(
                address_match
                or expected_names.intersection(observed_names)
            )

            pose = pose_from_mapping(data.get("currentPose"))
            if pose is None:
                return
            self.pose_sources["slam_info"] = (time.monotonic(), pose)
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    def best_pose(self) -> tuple[dict[str, float] | None, str | None, float | None]:
        now = time.monotonic()
        candidates = []

        # Only these sources are known to use the selected saved-map frame.
        # Fusion and pelvis odometry require an explicit map-frame anchor first.
        for source in ("slam_info", "relocation_odom"):
            sample = self.pose_sources.get(source)
            if sample is not None:
                received, pose = sample
                candidates.append((received, source, pose))

        if not candidates:
            return None, None, None

        received, source, pose = max(candidates, key=lambda item: item[0])
        age = max(0.0, now - received)
        return dict(pose), source, age

    def extract_xyz(self, message: PointCloud2) -> bytes:
        fields = {field.name: field for field in message.fields}
        required = [fields.get(name) for name in ("x", "y", "z")]
        if any(field is None for field in required):
            raise ValueError("PointCloud2 does not contain x/y/z fields")
        if any(field.datatype != PointField.FLOAT32 for field in required):
            raise ValueError("PointCloud2 x/y/z fields are not FLOAT32")

        width = int(message.width)
        height = max(1, int(message.height))
        total = width * height
        if total <= 0 or int(message.point_step) <= 0:
            return b""

        row_step = int(message.row_step) or width * int(message.point_step)
        endian = ">" if message.is_bigendian else "<"
        unpack_float = struct.Struct(endian + "f")
        step = max(1, math.ceil(total / self.maximum_points))
        output = array("f")

        raw = message.data
        for index in range(0, total, step):
            row = index // width
            column = index % width
            base = row * row_step + column * int(message.point_step)

            try:
                x = unpack_float.unpack_from(raw, base + required[0].offset)[0]
                y = unpack_float.unpack_from(raw, base + required[1].offset)[0]
                z = unpack_float.unpack_from(raw, base + required[2].offset)[0]
            except (struct.error, IndexError):
                continue

            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if max(abs(x), abs(y), abs(z)) > 100.0:
                continue

            output.extend((x, y, z))

        if sys.byteorder != "little":
            output.byteswap()
        return output.tobytes()

    def relocation_cloud_callback(self, message: PointCloud2) -> None:
        try:
            payload = self.extract_xyz(message)
            if not payload:
                return
            atomic_bytes(self.cloud_path, payload)
            self.cloud_sequence += 1
            self.cloud_received_monotonic = time.monotonic()
            self.cloud_source = "/unitree/slam_relocation/points"
            self.cloud_points = len(payload) // 12
            self.cloud_error = None
        except Exception as exc:
            self.cloud_error = str(exc)

    def poll_initialize_request(self) -> None:
        try:
            stat = self.initialize_path.stat()
        except OSError:
            return

        if stat.st_mtime_ns == self.initialize_last_mtime_ns:
            return
        self.initialize_last_mtime_ns = stat.st_mtime_ns

        try:
            if stat.st_size <= 0 or stat.st_size > 8192:
                raise ValueError("invalid initialization request size")

            request = json.loads(
                self.initialize_path.read_text(encoding="utf-8")
            )
            if (
                not isinstance(request, dict)
                or request.get("schema") != INITIALIZE_SCHEMA
            ):
                raise ValueError("invalid initialization request schema")

            request_id = request.get("request_id")
            if (
                not isinstance(request_id, int)
                or request_id < 1
                or request_id > 2147483647
            ):
                raise ValueError("invalid initialization request ID")

            pose = request.get("pose")
            if not isinstance(pose, dict):
                raise ValueError("pose must be an object")

            x = finite(pose.get("x"))
            y = finite(pose.get("y"))
            yaw = finite(pose.get("yaw"))
            if x is None or y is None or yaw is None:
                raise ValueError("pose must contain finite x, y and yaw")
            if abs(x) > 100.0 or abs(y) > 100.0:
                raise ValueError("initial position exceeds the map limit")

            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            if not self.native_map_path.is_file():
                raise FileNotFoundError(
                    f"native SLAM map unavailable: {self.native_map_path}"
                )

            now = time.monotonic()
            self.initialization = {
                "state": "PUBLISHED",
                "request_id": request_id,
                "requested_pose": {
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                },
                "response": None,
                "error": None,
            }
            self.initialization_started_monotonic = now

            # A pose received before this request must never be mistaken for
            # confirmation of the newly requested initialization.
            self.pose_sources.clear()
            self.localized_seen = False
            self.native_map_address = None
            self.native_pcd_name = None
            self.native_map_observed = False
            self.native_map_matches = False

            half_yaw = yaw * 0.5
            parameter = {
                "data": {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "q_x": 0.0,
                    "q_y": 0.0,
                    "q_z": math.sin(half_yaw),
                    "q_w": math.cos(half_yaw),
                    "address": str(self.native_map_path),
                }
            }

            message = UnitreeRequest()
            message.header.identity.id = request_id
            message.header.identity.api_id = INITIALIZE_API_ID
            message.header.lease.id = 0
            message.header.policy.priority = 1
            message.header.policy.noreply = False
            message.parameter = json.dumps(
                parameter,
                separators=(",", ":"),
            )

            self.slam_request_publisher.publish(message)
            self.get_logger().info(
                "Published SLAM initial pose "
                f"request={request_id} x={x:.3f} y={y:.3f} "
                f"yaw={yaw:.3f}"
            )
        except Exception as exc:
            self.initialization = {
                "state": "REJECTED",
                "request_id": None,
                "requested_pose": None,
                "response": None,
                "error": str(exc),
            }
            self.initialization_started_monotonic = time.monotonic()
            self.get_logger().error(
                f"Rejected SLAM initialization request: {exc}"
            )

    def slam_response_callback(self, message: UnitreeResponse) -> None:
        initialization = self.initialization
        if initialization.get("state") != "PUBLISHED":
            return

        request_id = initialization.get("request_id")
        if (
            int(message.header.identity.api_id) != INITIALIZE_API_ID
            or int(message.header.identity.id) != request_id
        ):
            return

        status_code = int(message.header.status.code)
        try:
            data = json.loads(message.data) if message.data else {}
        except json.JSONDecodeError:
            data = {}

        error_code = data.get("errorCode")
        succeeded = (
            status_code == 0
            and data.get("succeed") is True
            and error_code in (None, 0)
        )

        initialization["response"] = {
            "status_code": status_code,
            "data": data,
        }

        if succeeded:
            initialization["state"] = "ACCEPTED"
            initialization["error"] = None
            self.get_logger().info(
                f"SLAM initial pose accepted request={request_id}"
            )
        else:
            initialization["state"] = "REJECTED"
            initialization["error"] = str(
                data.get("info")
                or f"API 1804 response status={status_code}"
            )
            self.get_logger().error(
                "SLAM initial pose rejected "
                f"request={request_id}: "
                f"{initialization['error']}"
            )

    def initialization_snapshot(self, *, localized: bool) -> dict[str, Any]:
        state = str(self.initialization.get("state") or "IDLE")
        age = None

        if self.initialization_started_monotonic:
            age = max(
                0.0,
                time.monotonic()
                - self.initialization_started_monotonic,
            )

        if state in {"PUBLISHED", "ACCEPTED"}:
            if localized:
                state = "LOCALIZED"
                self.initialization["state"] = state
            elif age is not None and age > INITIALIZE_TIMEOUT_S:
                state = "TIMEOUT"
                self.initialization["state"] = state
                self.initialization["error"] = (
                    "No fresh saved-map pose arrived within "
                    f"{INITIALIZE_TIMEOUT_S:.0f} seconds"
                )

        return {
            "state": state,
            "request_id": self.initialization.get("request_id"),
            "requested_pose": self.initialization.get("requested_pose"),
            "response": self.initialization.get("response"),
            "error": self.initialization.get("error"),
            "age_s": round(age, 3) if age is not None else None,
        }

    def publish_status(self) -> None:
        now = time.monotonic()
        pose, pose_source, pose_age = self.best_pose()
        pose_available = bool(
            self.native_map_matches
            and pose is not None
            and pose_source in {"slam_info", "relocation_odom"}
        )
        pose_fresh = bool(
            pose_available
            and pose_age is not None
            and pose_age <= POSE_MAX_AGE_S
        )
        localized = pose_fresh
        if localized:
            self.localized_seen = True

        cloud_age = (
            max(0.0, now - self.cloud_received_monotonic)
            if self.cloud_received_monotonic
            else None
        )
        cloud_online = cloud_age is not None and cloud_age <= CLOUD_MAX_AGE_S

        if localized:
            state = "LOCALIZED"
        elif self.native_map_observed and not self.native_map_matches:
            state = "MAP_MISMATCH"
        elif pose_available:
            state = "STALE"
        elif self.localized_seen:
            state = "LOST"
        else:
            state = "UNLOCALIZED"

        initialization = self.initialization_snapshot(
            localized=localized
        )

        atomic_json(
            self.status_path,
            {
                "schema": SCHEMA,
                "worker_pid": os.getpid(),
                "state": state,
                "localized": localized,
                "pose_available": pose_available,
                "pose_fresh": pose_fresh,
                "uptime_s": round(now - self.started_monotonic, 3),
                "pose": pose,
                "pose_source": pose_source,
                "pose_age_s": (
                    round(pose_age, 3) if pose_age is not None else None
                ),
                "initialization": initialization,
                "map": self.map_info,
                "native_map": {
                    "expected_address": str(self.native_map_path),
                    "observed_address": self.native_map_address,
                    "observed_pcd_name": self.native_pcd_name,
                    "identity_observed": self.native_map_observed,
                    "matches": self.native_map_matches,
                },
                "cloud": {
                    "online": cloud_online,
                    "source": self.cloud_source,
                    "sequence": self.cloud_sequence,
                    "points": self.cloud_points,
                    "age_s": (
                        round(cloud_age, 3) if cloud_age is not None else None
                    ),
                    "path": str(self.cloud_path),
                    "error": self.cloud_error,
                },
            },
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map",
        default=os.environ.get("G1_DASHBOARD_SLAM_MAP", str(DEFAULT_MAP)),
    )
    parser.add_argument(
        "--native-map",
        default=os.environ.get(
            "G1_DASHBOARD_SLAM_NATIVE_MAP",
            str(DEFAULT_NATIVE_MAP),
        ),
    )
    parser.add_argument(
        "--state-dir",
        default=str(DEFAULT_STATE_DIR),
    )
    parser.add_argument(
        "--maximum-points",
        type=int,
        default=int(
            os.environ.get(
                "G1_DASHBOARD_SLAM_MAX_POINTS",
                str(DEFAULT_MAX_POINTS),
            )
        ),
    )
    args = parser.parse_args()

    map_path = Path(args.map).expanduser().resolve()
    native_map_path = Path(args.native_map).expanduser()
    state_dir = Path(args.state_dir).expanduser()

    rclpy.init()
    node = SlamSnapshotWorker(
        map_path=map_path,
        native_map_path=native_map_path,
        state_dir=state_dir,
        maximum_points=args.maximum_points,
    )

    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
