#!/usr/bin/env python3
"""G1 dashboard bridge v1.6.2 — point-cloud orbit + 2-D top occupancy + verified service actions.

Safety boundary:
- Receives controller telemetry only from localhost UDP (127.0.0.1:8765).
- Uses only the Python standard library: no FastAPI/Uvicorn/WebSocket packages.
- Does NOT import Unitree DDS libraries and does NOT publish DDS commands.
- Keeps authenticated start/stop lifecycle requests for one exact validated controller.
- Couples a root-helper-managed Inspire service to the managed controller lifecycle.
- Adds authenticated teleimager process start/stop; browser WebRTC remains separate.
- Step 5.1 adds an authenticated XR action request endpoint. The bridge does not
  decide robot state or publish DDS: it forwards only a whitelisted operation to
  the controller's loopback action socket, where the controller re-validates it.
- Step 5.2 adds authenticated, explicitly allowlisted service requests forwarded
  to a separate g1_xr worker. The bridge itself still imports no Unitree DDS.
- Browser/client disconnects have no effect on an already running controller.
"""
from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import socket
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

from g1_dashboard_process_manager import ControllerProcessManager
from g1_dashboard_service_client import ServiceActionClient
from g1_dashboard_service_policy import classify_service, load_policy, public_policy

SCHEMA = "g1_dashboard.telemetry.v1"
BRIDGE_VERSION = "g1_dashboard_bridge.v1.10.0-independent-robot-stream"
SYSTEM_SCHEMA = "g1_dashboard.system.v1"
ROBOT_SCHEMA = "g1_dashboard.robot_telemetry.v1"
STATIC_DIR = Path(__file__).resolve().parent / "static"

SLAM_STATE_DIR = Path(
    os.environ.get(
        "G1_DASHBOARD_SLAM_STATE_DIR",
        f"/tmp/g1_dashboard_slam_{os.getuid()}",
    )
)
SLAM_STATUS_PATH = SLAM_STATE_DIR / "status.json"
SLAM_CLOUD_PATH = SLAM_STATE_DIR / "live_cloud_f32.bin"
SLAM_INITIALIZE_PATH = SLAM_STATE_DIR / "initialize_request.json"
SLAM_INITIALIZE_SCHEMA = "g1_dashboard.slam.initialize.v1"
SLAM_MAP_PATH = Path(
    os.environ.get(
        "G1_DASHBOARD_SLAM_MAP",
        str(Path.home() / "g1_ws/map/harta_buna_2707.pcd"),
    )
).expanduser()


def slam_status_snapshot() -> dict[str, Any]:
    fallback = {
        "schema": "g1_dashboard.slam.v1",
        "worker_online": False,
        "state": "OFFLINE",
        "localized": False,
        "pose": None,
        "pose_source": None,
        "map": {
            "path": str(SLAM_MAP_PATH),
            "available": SLAM_MAP_PATH.is_file(),
        },
        "cloud": {
            "online": False,
            "points": 0,
            "sequence": 0,
        },
    }

    try:
        stat = SLAM_STATUS_PATH.stat()
        if stat.st_size <= 0 or stat.st_size > 65536:
            raise ValueError("invalid SLAM status size")
        payload = json.loads(
            SLAM_STATUS_PATH.read_text(encoding="utf-8")
        )
        if (
            not isinstance(payload, dict)
            or payload.get("schema") != "g1_dashboard.slam.v1"
        ):
            raise ValueError("invalid SLAM status schema")
    except Exception as exc:
        fallback["error"] = str(exc)
        return fallback

    age = max(0.0, time.time() - stat.st_mtime)
    payload["status_age_s"] = round(age, 3)
    payload["worker_online"] = age <= 2.0
    if not payload["worker_online"]:
        payload["state"] = "OFFLINE"
        payload["localized"] = False
        cloud = payload.get("cloud")
        if isinstance(cloud, dict):
            cloud["online"] = False
    return payload


def write_slam_initialize_request(
    payload: dict[str, Any],
) -> dict[str, Any]:
    status = slam_status_snapshot()
    if not status.get("worker_online"):
        raise RuntimeError("SLAM worker is offline")

    current = status.get("initialization")
    if (
        isinstance(current, dict)
        and current.get("state") in {"PUBLISHED", "ACCEPTED"}
    ):
        raise RuntimeError("a SLAM initialization request is already active")

    x = finite_number(payload.get("x"))
    y = finite_number(payload.get("y"))
    yaw = finite_number(payload.get("yaw"))
    if x is None or y is None or yaw is None:
        raise ValueError("x, y and yaw must be finite numbers")
    if abs(x) > 100.0 or abs(y) > 100.0:
        raise ValueError("initial position exceeds the map limit")

    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    request_id = int(time.time_ns() % 2147483646) + 1
    request = {
        "schema": SLAM_INITIALIZE_SCHEMA,
        "request_id": request_id,
        "created_unix_ns": time.time_ns(),
        "pose": {
            "x": x,
            "y": y,
            "yaw": yaw,
        },
    }

    SLAM_STATE_DIR.mkdir(parents=True, exist_ok=True)
    temporary = SLAM_INITIALIZE_PATH.with_name(
        f".{SLAM_INITIALIZE_PATH.name}."
        f"{os.getpid()}.{threading.get_ident()}.tmp"
    )
    temporary.write_text(
        json.dumps(request, separators=(",", ":"), sort_keys=True),
        encoding="utf-8",
    )
    os.replace(temporary, SLAM_INITIALIZE_PATH)
    return request


def now_s() -> float:
    return time.time()


def finite_number(value: Any) -> float | None:
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def nested(data: dict[str, Any] | None, *keys: str, default: Any = None) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(key)
    return default if cur is None else cur


def unity_telemetry_snapshot(
    state: Any,
    system_state: Any,
) -> dict[str, Any]:
    """Compact read-only telemetry for the Unity HUD."""

    telemetry_envelope = state.envelope()
    system_envelope = system_state.envelope()

    bridge = telemetry_envelope.get("bridge")
    if not isinstance(bridge, dict):
        bridge = {}

    telemetry = telemetry_envelope.get("telemetry")
    if not isinstance(telemetry, dict):
        telemetry = {}

    monitor = system_envelope.get("monitor")
    if not isinstance(monitor, dict):
        monitor = {}

    system = system_envelope.get("system")
    if not isinstance(system, dict):
        system = {}

    def maximum_absolute(values: Any) -> float | None:
        if not isinstance(values, list):
            return None

        finite = [
            abs(number)
            for value in values
            if (number := finite_number(value)) is not None
        ]

        return max(finite) if finite else None

    robot = nested(telemetry, "robot", default={})
    if not isinstance(robot, dict):
        robot = {}

    joint_names = robot.get("joint_names")
    if not isinstance(joint_names, list):
        joint_names = []

    temperatures = robot.get("temperatures_c")
    if not isinstance(temperatures, list):
        temperatures = []

    hottest_temperature = None
    hottest_joint = None

    for index, pair in enumerate(temperatures):
        if not isinstance(pair, list):
            continue

        values = [
            temperature
            for value in pair
            if (temperature := finite_number(value)) is not None
        ]

        if not values:
            continue

        candidate = max(values)

        if (
            hottest_temperature is None
            or candidate > hottest_temperature
        ):
            hottest_temperature = candidate
            hottest_joint = (
                str(joint_names[index])
                if index < len(joint_names)
                else f"motor_{index}"
            )

    motor_states = robot.get("motor_state")
    if not isinstance(motor_states, list):
        motor_states = []

    motor_fault_count = sum(
        1
        for value in motor_states
        if value is not None
        and value != 0
    )

    max_joint_speed = maximum_absolute(
        robot.get("measured_dq_rps")
    )

    max_estimated_torque = maximum_absolute(
        robot.get("tau_est")
    )

    arm_error_rad = finite_number(
        nested(
            telemetry,
            "arms",
            "max_abs_published_error_rad",
        )
    )

    arm_error_deg = (
        arm_error_rad * 180.0 / math.pi
        if arm_error_rad is not None
        else None
    )

    velocity = nested(
        system,
        "base_sensing",
        "odometry",
        "velocity_mps",
        default=[],
    )

    base_speed = None
    if isinstance(velocity, list):
        components = [
            number
            for value in velocity[:3]
            if (number := finite_number(value)) is not None
        ]

        if len(components) == 3:
            base_speed = math.sqrt(
                sum(value * value for value in components)
            )

    yaw_rate = finite_number(
        nested(
            telemetry,
            "motion",
            "lower_body",
            "yaw_rate_rps",
        )
    )

    if yaw_rate is None:
        yaw_rate = finite_number(
            nested(
                system,
                "base_sensing",
                "odometry",
                "yaw_speed_rps",
            )
        )

    return {
        "schema": "g1_dashboard.unity_telemetry.v1",
        "generated_unix_time_s": now_s(),
        "connection": {
            "online": bool(
                bridge.get("telemetry_online")
            ),
            "packet_age_s": finite_number(
                bridge.get("packet_age_s")
            ),
            "packet_count": bridge.get("packet_count"),
            "invalid_count": bridge.get("invalid_count"),
        },
        "control": {
            "state": nested(
                telemetry,
                "mode",
                "state",
            ),
            "main_loop_hz": finite_number(
                nested(
                    telemetry,
                    "controller",
                    "main_loop_hz",
                )
            ),
            "arm_ownership": finite_number(
                nested(
                    telemetry,
                    "mode",
                    "arm_ownership_weight",
                )
            ),
            "lowstate_ok": bool(
                nested(
                    telemetry,
                    "health",
                    "lowstate",
                    "ok",
                    default=False,
                )
            ),
            "lowstate_age_s": finite_number(
                nested(
                    telemetry,
                    "health",
                    "lowstate",
                    "age_s",
                )
            ),
            "xr_ok": bool(
                nested(
                    telemetry,
                    "health",
                    "xr",
                    "ok",
                    default=False,
                )
            ),
            "xr_reason": nested(
                telemetry,
                "health",
                "xr",
                "reason",
            ),
            "safety_fault": nested(
                telemetry,
                "mode",
                "safety_fault_reason",
            ),
            "tracking_hold": nested(
                telemetry,
                "mode",
                "tracking_hold_reason",
            ),
            "tracking_guard": bool(
                nested(
                    telemetry,
                    "health",
                    "tracking_guard",
                    "active",
                    default=False,
                )
            ),
        },
        "motion": {
            "base_speed_mps": base_speed,
            "yaw_rate_rps": yaw_rate,
            "max_joint_speed_rps": max_joint_speed,
        },
        "arms": {
            "max_tracking_error_deg": arm_error_deg,
            "max_estimated_torque_nm": max_estimated_torque,
            "publisher_ok": not bool(
                nested(
                    telemetry,
                    "health",
                    "arm_publisher",
                    "error",
                )
            ),
            "publisher_error": nested(
                telemetry,
                "health",
                "arm_publisher",
                "error",
            ),
        },
        "motors": {
            "hottest_joint": hottest_joint,
            "hottest_temperature_c": hottest_temperature,
            "fault_count": motor_fault_count,
        },
        "hands": {
            "mode": nested(
                telemetry,
                "hands",
                "mode",
            ),
            "tracking_valid": bool(
                nested(
                    telemetry,
                    "hands",
                    "tracking_valid",
                    default=False,
                )
            ),
            "tracking_reason": nested(
                telemetry,
                "hands",
                "tracking_reason",
            ),
            "feedback_fault": bool(
                nested(
                    telemetry,
                    "health",
                    "finger_worker",
                    "feedback_fault",
                    default=False,
                )
            ),
            "feedback_age_s": finite_number(
                nested(
                    telemetry,
                    "health",
                    "finger_worker",
                    "feedback_age_s",
                )
            ),
        },
        "computer": {
            "online": bool(
                monitor.get("online")
            ),
            "cpu_used_pct": finite_number(
                nested(
                    system,
                    "host",
                    "cpu",
                    "used_pct",
                )
            ),
            "ram_used_pct": finite_number(
                nested(
                    system,
                    "host",
                    "memory",
                    "used_pct",
                )
            ),
            "disk_used_pct": finite_number(
                nested(
                    system,
                    "host",
                    "disk_root",
                    "used_pct",
                )
            ),
            "maximum_temperature_c": finite_number(
                nested(
                    system,
                    "host",
                    "thermal",
                    "max_c",
                )
            ),
            "imu_temperature_c": finite_number(
                nested(
                    system,
                    "base_sensing",
                    "imu",
                    "temperature_c",
                )
            ),
            "network_state": nested(
                system,
                "host",
                "network",
                "operstate",
            ),
            "uptime_s": finite_number(
                nested(
                    system,
                    "host",
                    "uptime_s",
                )
            ),
        },
    }


class BridgeState:
    def __init__(self, *, stale_after_s: float, event_limit: int = 250) -> None:
        self.stale_after_s = float(stale_after_s)
        self._lock = threading.RLock()
        self.latest: dict[str, Any] | None = None
        self.latest_received_monotonic: float | None = None
        self.packet_count = 0
        self.invalid_count = 0
        self.last_source: str | None = None
        self.events: deque[dict[str, Any]] = deque(maxlen=event_limit)
        self._previous_summary: dict[str, Any] | None = None
        self._stale_latched = True

        self.robot_latest: dict[str, Any] | None = None
        self.robot_received_monotonic: float | None = None
        self.robot_packet_count = 0
        self.robot_invalid_count = 0
        self.robot_last_source: str | None = None
        self._robot_stale_latched = True

    def _robot_packet_age_locked(self) -> float | None:
        if self.robot_received_monotonic is None:
            return None

        return max(
            0.0,
            time.monotonic()
            - self.robot_received_monotonic,
        )

    def accept_robot(
        self,
        packet: dict[str, Any],
        source: str,
    ) -> None:
        with self._lock:
            self.robot_latest = packet
            self.robot_received_monotonic = (
                time.monotonic()
            )
            self.robot_packet_count += 1
            self.robot_last_source = source

            if self._robot_stale_latched:
                self._robot_stale_latched = False
                self.add_event(
                    "info",
                    "robot",
                    "Independent robot telemetry "
                    "link online",
                    seq=packet.get("sequence"),
                )

    def reject_robot(self, reason: str) -> None:
        with self._lock:
            self.robot_invalid_count += 1

            if (
                self.robot_invalid_count <= 5
                or self.robot_invalid_count % 100 == 0
            ):
                self.add_event(
                    "warning",
                    "robot",
                    "Rejected robot telemetry: "
                    f"{reason}",
                )

    def _merged_telemetry_locked(
        self,
    ) -> dict[str, Any] | None:
        controller = self.latest
        robot_age = self._robot_packet_age_locked()

        robot_online = (
            robot_age is not None
            and robot_age <= self.stale_after_s
            and isinstance(self.robot_latest, dict)
        )

        if (
            not isinstance(controller, dict)
            and not robot_online
        ):
            return None

        merged = (
            dict(controller)
            if isinstance(controller, dict)
            else {
                "schema": SCHEMA,
                "sequence": None,
                "unix_time_s": None,
            }
        )

        if robot_online:
            robot_packet = self.robot_latest
            robot = robot_packet.get("robot")

            if isinstance(robot, dict):
                merged["robot"] = robot

            if not isinstance(controller, dict):
                merged["sequence"] = (
                    robot_packet.get("sequence")
                )
                merged["unix_time_s"] = (
                    robot_packet.get("unix_time_s")
                )

        return merged

    def _packet_age_locked(self) -> float | None:
        if self.latest_received_monotonic is None:
            return None
        return max(0.0, time.monotonic() - self.latest_received_monotonic)

    def packet_age_s(self) -> float | None:
        with self._lock:
            return self._packet_age_locked()

    def telemetry_ok(self) -> bool:
        with self._lock:
            age = self._packet_age_locked()
            return age is not None and age <= self.stale_after_s

    def add_event(self, level: str, category: str, message: str, *, seq: int | None = None) -> None:
        self.events.appendleft({
            "id": f"{int(now_s()*1000)}-{len(self.events)}",
            "unix_time_s": now_s(),
            "level": level,
            "category": category,
            "message": message,
            "sequence": seq,
        })

    @staticmethod
    def validate_packet(packet: Any) -> tuple[bool, str]:
        if not isinstance(packet, dict):
            return False, "packet is not an object"
        if packet.get("schema") != SCHEMA:
            return False, f"unexpected schema {packet.get('schema')!r}"
        if not isinstance(packet.get("sequence"), int):
            return False, "sequence is not an integer"
        for key in ("controller", "mode", "health", "motion", "tracking", "arms", "hands", "camera"):
            if not isinstance(packet.get(key), dict):
                return False, f"missing/invalid top-level object: {key}"
        return True, "ok"

    def _summary(self, packet: dict[str, Any]) -> dict[str, Any]:
        return {
            "state": nested(packet, "mode", "state"),
            "shutdown_pending": nested(packet, "mode", "shutdown_pending"),
            "safety_fault": nested(packet, "mode", "safety_fault_reason"),
            "tracking_hold": nested(packet, "mode", "tracking_hold_reason"),
            "xr_ok": nested(packet, "health", "xr", "ok"),
            "xr_reason": nested(packet, "health", "xr", "reason"),
            "lowstate_ok": nested(packet, "health", "lowstate", "ok"),
            "guard": nested(packet, "health", "tracking_guard", "active"),
            "guard_l": nested(packet, "health", "tracking_guard", "rejected_left"),
            "guard_r": nested(packet, "health", "tracking_guard", "rejected_right"),
            "finger_phase": nested(packet, "health", "finger_worker", "phase"),
            "finger_error": nested(packet, "health", "finger_worker", "error"),
            "finger_feedback_fault": nested(packet, "health", "finger_worker", "feedback_fault"),
            "hands_mode": nested(packet, "hands", "mode"),
            "arm_error": nested(packet, "health", "arm_publisher", "error"),
        }

    def _detect_events(self, packet: dict[str, Any]) -> None:
        seq = packet.get("sequence") if isinstance(packet.get("sequence"), int) else None
        cur = self._summary(packet)
        prev = self._previous_summary
        if prev is None:
            self.add_event("info", "bridge", f"Telemetry stream acquired at sequence {seq}", seq=seq)
            self.add_event("info", "mode", f"Initial controller state: {cur['state']}", seq=seq)
            self._previous_summary = cur
            return

        if cur["state"] != prev["state"]:
            level = "error" if cur["state"] == "SAFETY_FAULT_HOLD" else "warning" if cur["state"] == "XR_TRACKING_HOLD" else "info"
            self.add_event(level, "mode", f"{prev['state']} → {cur['state']}", seq=seq)
        if cur["xr_ok"] != prev["xr_ok"]:
            self.add_event("info" if cur["xr_ok"] else "warning", "tracking", "XR tracking became healthy" if cur["xr_ok"] else f"XR tracking unavailable: {cur['xr_reason']}", seq=seq)
        if cur["lowstate_ok"] != prev["lowstate_ok"]:
            self.add_event("info" if cur["lowstate_ok"] else "error", "robot", "LowState recovered" if cur["lowstate_ok"] else "LowState became stale/unavailable", seq=seq)
        if cur["guard"] != prev["guard"]:
            if cur["guard"]:
                sides = []
                if cur["guard_l"]: sides.append("L")
                if cur["guard_r"]: sides.append("R")
                self.add_event("warning", "tracking", f"Bimanual tracking guard active ({'/'.join(sides) or 'pair'})", seq=seq)
            else:
                self.add_event("info", "tracking", "Bimanual tracking guard cleared", seq=seq)
        if cur["hands_mode"] != prev["hands_mode"]:
            self.add_event("info", "hands", f"Finger mode {prev['hands_mode']} → {cur['hands_mode']}", seq=seq)
        if cur["finger_phase"] != prev["finger_phase"]:
            self.add_event("info", "hands", f"Finger worker phase: {cur['finger_phase']}", seq=seq)
        if cur["finger_feedback_fault"] != prev["finger_feedback_fault"]:
            self.add_event("error" if cur["finger_feedback_fault"] else "info", "hands", "Inspire feedback fault" if cur["finger_feedback_fault"] else "Inspire feedback recovered", seq=seq)
        if cur["finger_error"] and cur["finger_error"] != prev["finger_error"]:
            self.add_event("error", "hands", f"Finger worker error: {cur['finger_error']}", seq=seq)
        if cur["arm_error"] and cur["arm_error"] != prev["arm_error"]:
            self.add_event("error", "arms", f"Arm publisher error: {cur['arm_error']}", seq=seq)
        if cur["safety_fault"] and cur["safety_fault"] != prev["safety_fault"]:
            self.add_event("error", "safety", f"Safety fault: {cur['safety_fault']}", seq=seq)
        if cur["tracking_hold"] and cur["tracking_hold"] != prev["tracking_hold"]:
            self.add_event("warning", "tracking", f"Tracking hold: {cur['tracking_hold']}", seq=seq)
        if cur["shutdown_pending"] and not prev["shutdown_pending"]:
            self.add_event("warning", "mode", "Controlled shutdown requested", seq=seq)
        self._previous_summary = cur

    def accept(self, packet: dict[str, Any], source: str) -> None:
        with self._lock:
            self.latest = packet
            self.latest_received_monotonic = time.monotonic()
            self.packet_count += 1
            self.last_source = source
            self._detect_events(packet)
            if self._stale_latched:
                self._stale_latched = False
                self.add_event("info", "bridge", "Telemetry link online", seq=packet.get("sequence"))

    def update_stale_event(self) -> None:
        with self._lock:
            age = self._packet_age_locked()
            stale = (
                age is None
                or age > self.stale_after_s
            )

            if stale and not self._stale_latched:
                self._stale_latched = True
                self.add_event(
                    "warning",
                    "bridge",
                    "Telemetry link stale "
                    f"(> {self.stale_after_s:.1f}s "
                    "without packets)",
                )

            robot_age = (
                self._robot_packet_age_locked()
            )
            robot_stale = (
                robot_age is None
                or robot_age > self.stale_after_s
            )

            if (
                robot_stale
                and not self._robot_stale_latched
            ):
                self._robot_stale_latched = True
                self.add_event(
                    "warning",
                    "robot",
                    "Independent robot telemetry "
                    "link stale "
                    f"(> {self.stale_after_s:.1f}s "
                    "without packets)",
                )

    def bridge_meta_locked(self) -> dict[str, Any]:
        age = self._packet_age_locked()
        robot_age = self._robot_packet_age_locked()

        return {
            "version": BRIDGE_VERSION,
            "schema": SCHEMA,
            "transport": "http-polling",
            "telemetry_online": (
                age is not None
                and age <= self.stale_after_s
            ),
            "packet_age_s": finite_number(age),
            "packet_count": self.packet_count,
            "invalid_count": self.invalid_count,
            "last_source": self.last_source,
            "robot_online": (
                robot_age is not None
                and robot_age <= self.stale_after_s
            ),
            "robot_packet_age_s":
                finite_number(robot_age),
            "robot_packet_count":
                self.robot_packet_count,
            "robot_invalid_count":
                self.robot_invalid_count,
            "robot_last_source":
                self.robot_last_source,
            "stale_after_s": self.stale_after_s,
        }

    def envelope(self) -> dict[str, Any]:
        with self._lock:
            self.update_stale_event()
            return {
                "type": "telemetry",
                "bridge": self.bridge_meta_locked(),
                "telemetry": self._merged_telemetry_locked(),
                "events": list(self.events)[:100],
            }

    def pose_envelope(self) -> dict[str, Any]:
        """Small latest-only pose payload for the browser's 30 Hz render path."""
        with self._lock:
            age = self._packet_age_locked()
            t = self._merged_telemetry_locked()

            robot_age = (
                self._robot_packet_age_locked()
            )
            robot_packet = (
                self.robot_latest
                if (
                    robot_age is not None
                    and robot_age
                        <= self.stale_after_s
                    and isinstance(
                        self.robot_latest,
                        dict,
                    )
                )
                else None
            )

            if not isinstance(t, dict):
                return {
                    "type": "pose",
                    "bridge": {
                        "telemetry_online": False,
                        "packet_age_s": finite_number(age),
                        "server_unix_time_s": now_s(),
                    },
                    "sequence": None,
                    "source_unix_time_s": None,
                    "robot": None,
                    "arms": None,
                    "hands": None,
                }
            robot = t.get("robot") if isinstance(t.get("robot"), dict) else {}
            arms = t.get("arms") if isinstance(t.get("arms"), dict) else {}
            hands = t.get("hands") if isinstance(t.get("hands"), dict) else {}
            return {
                "type": "pose",
                "bridge": {
                    "telemetry_online": age is not None and age <= self.stale_after_s,
                    "packet_age_s": finite_number(age),
                    "server_unix_time_s": now_s(),
                },
                "sequence": (
                    robot_packet.get("sequence")
                    if robot_packet is not None
                    else t.get("sequence")
                ),
                "source_unix_time_s": (
                    robot_packet.get("unix_time_s")
                    if robot_packet is not None
                    else t.get("unix_time_s")
                ),
                "robot": {
                    "mode_machine": robot.get("mode_machine"),
                    "measured_q_rad": robot.get("measured_q_rad"),
                },
                "arms": {
                    "published_q_rad": arms.get("published_q_rad"),
                },
                "hands": {
                    "current_left": hands.get("current_left"),
                    "current_right": hands.get("current_right"),
                    "feedback_state": hands.get("feedback_state"),
                },
            }


class UdpTelemetryThread(threading.Thread):
    def __init__(self, state: BridgeState, host: str, port: int) -> None:
        super().__init__(name="g1-dashboard-udp", daemon=True)
        self.state = state
        self.host = host
        self.port = int(port)
        self._stop_event = threading.Event()
        self._sock: socket.socket | None = None

    def stop(self) -> None:
        self._stop_event.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock = sock
        sock.settimeout(0.5)
        sock.bind((self.host, self.port))
        self.state.add_event("info", "bridge", f"Bridge listening for telemetry on udp://{self.host}:{self.port}")
        while not self._stop_event.is_set():
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                self.state.update_stale_event()
                continue
            except OSError:
                break
            try:
                packet = json.loads(data.decode("utf-8"))
            except Exception as exc:
                with self.state._lock:
                    self.state.invalid_count += 1
                    if self.state.invalid_count <= 5 or self.state.invalid_count % 100 == 0:
                        self.state.add_event("warning", "bridge", f"Rejected malformed UDP telemetry: {exc}")
                continue
            ok, reason = self.state.validate_packet(packet)
            if not ok:
                with self.state._lock:
                    self.state.invalid_count += 1
                    if self.state.invalid_count <= 5 or self.state.invalid_count % 100 == 0:
                        self.state.add_event("warning", "bridge", f"Rejected UDP telemetry: {reason}")
                continue
            self.state.accept(packet, f"{addr[0]}:{addr[1]}")


class UdpRobotTelemetryThread(threading.Thread):
    def __init__(
        self,
        state: BridgeState,
        host: str,
        port: int,
    ) -> None:
        super().__init__(
            name="g1-dashboard-robot-udp",
            daemon=True,
        )
        self.state = state
        self.host = host
        self.port = int(port)
        self._stop_event = threading.Event()
        self._sock: socket.socket | None = None

    def stop(self) -> None:
        self._stop_event.set()

        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass

    @staticmethod
    def validate(
        packet: Any,
    ) -> tuple[bool, str]:
        if not isinstance(packet, dict):
            return False, "packet is not an object"

        if packet.get("schema") != ROBOT_SCHEMA:
            return (
                False,
                "unexpected schema "
                f"{packet.get('schema')!r}",
            )

        if not isinstance(
            packet.get("sequence"),
            int,
        ):
            return (
                False,
                "sequence is not an integer",
            )

        robot = packet.get("robot")
        if not isinstance(robot, dict):
            return False, "robot is not an object"

        measured = robot.get("measured_q_rad")
        if (
            not isinstance(measured, list)
            or len(measured) != 29
        ):
            return (
                False,
                "measured_q_rad must contain "
                "29 joints",
            )

        return True, "ok"

    def run(self) -> None:
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
        self._sock = sock
        sock.settimeout(0.5)
        sock.bind((self.host, self.port))

        self.state.add_event(
            "info",
            "robot",
            "Bridge listening for independent "
            "robot telemetry on "
            f"udp://{self.host}:{self.port}",
        )

        while not self._stop_event.is_set():
            try:
                data, address = sock.recvfrom(65535)
            except socket.timeout:
                self.state.update_stale_event()
                continue
            except OSError:
                break

            try:
                packet = json.loads(
                    data.decode("utf-8")
                )
            except Exception as exc:
                self.state.reject_robot(
                    f"malformed JSON: {exc}"
                )
                continue

            valid, reason = self.validate(packet)
            if not valid:
                self.state.reject_robot(reason)
                continue

            self.state.accept_robot(
                packet,
                f"{address[0]}:{address[1]}",
            )


class SystemState:
    def __init__(self, *, stale_after_s: float = 3.0) -> None:
        self.stale_after_s = float(stale_after_s)
        self._lock = threading.RLock()
        self.latest: dict[str, Any] | None = None
        self.latest_received_monotonic: float | None = None
        self.packet_count = 0
        self.invalid_count = 0

    def accept(self, packet: dict[str, Any]) -> None:
        with self._lock:
            self.latest = packet
            self.latest_received_monotonic = time.monotonic()
            self.packet_count += 1

    def envelope(self) -> dict[str, Any]:
        with self._lock:
            age = None if self.latest_received_monotonic is None else max(0.0, time.monotonic() - self.latest_received_monotonic)
            online = age is not None and age <= self.stale_after_s
            return {
                "monitor": {
                    "online": online,
                    "packet_age_s": finite_number(age),
                    "packet_count": self.packet_count,
                    "invalid_count": self.invalid_count,
                    "stale_after_s": self.stale_after_s,
                },
                "system": self.latest,
            }


class UdpSystemThread(threading.Thread):
    def __init__(self, state: SystemState, host: str, port: int) -> None:
        super().__init__(name="g1-dashboard-system-udp", daemon=True)
        self.state = state
        self.host = host
        self.port = int(port)
        self._stop_event = threading.Event()
        self._sock: socket.socket | None = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock = sock
        sock.settimeout(0.5)
        sock.bind((self.host, self.port))
        while not self._stop_event.is_set():
            try:
                data, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                packet = json.loads(data.decode("utf-8"))
                if not isinstance(packet, dict) or packet.get("schema") != SYSTEM_SCHEMA or not isinstance(packet.get("sequence"), int):
                    raise ValueError("invalid system-monitor packet")
                self.state.accept(packet)
            except Exception:
                with self.state._lock:
                    self.state.invalid_count += 1


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "G1Dashboard/1.5.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: object) -> None:
        # /api/pose is intentionally high-rate; do not flood the terminal at 30 Hz.
        if (
            self.path.startswith("/api/pose")
            or self.path.startswith("/api/camera/pointcloud")
            or self.path.startswith("/api/slam/cloud")
        ):
            return
        print(f"HTTP {self.client_address[0]} - {fmt % args}")

    def _send_bytes(self, status: int, body: bytes, content_type: str, *, cache: bool = False) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "public, max-age=3600" if cache else "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _send_json(self, status: int, payload: Any) -> None:
        body = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._send_bytes(status, body, "application/json; charset=utf-8", cache=False)

    def _read_json_body(self, *, max_bytes: int = 65536) -> Any:
        content_type = self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
        if content_type != "application/json":
            raise ValueError("Content-Type must be application/json")
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length < 0 or length > max_bytes:
            raise ValueError(f"request body must be <= {max_bytes} bytes")
        raw = self.rfile.read(length) if length else b"{}"
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception as exc:
            raise ValueError(f"invalid JSON body: {exc}") from exc

    def _process_action_authorized(self) -> bool:
        import secrets
        expected: str = self.server.action_token  # type: ignore[attr-defined]
        supplied = self.headers.get("X-G1-Management-Key", "")
        return bool(expected) and bool(supplied) and secrets.compare_digest(supplied, expected)

    def _require_process_action_auth(self) -> bool:
        if not self.server.process_actions_enabled:  # type: ignore[attr-defined]
            self._send_json(HTTPStatus.FORBIDDEN, {"error": "controller process actions are disabled"})
            return False
        if not self._process_action_authorized():
            self._send_json(HTTPStatus.UNAUTHORIZED, {"error": "invalid or missing management key"})
            return False
        return True

    def _serve_file(self, path: Path, *, cache: bool = False) -> None:
        try:
            resolved = path.resolve(strict=True)
            root = STATIC_DIR.resolve(strict=True)
            resolved.relative_to(root)
        except Exception:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if not resolved.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = mimetypes.guess_type(str(resolved))[0] or "application/octet-stream"
        self._send_bytes(HTTPStatus.OK, resolved.read_bytes(), content_type, cache=cache)

    def do_HEAD(self) -> None:
        self.do_GET()

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        state: BridgeState = self.server.state  # type: ignore[attr-defined]
        system_state: SystemState = self.server.system_state  # type: ignore[attr-defined]

        if path == "/":
            self._serve_file(STATIC_DIR / "index.html")
            return
        if path.startswith("/static/"):
            rel = path[len("/static/"):]
            immutable_asset = rel.startswith("vendor/") or rel.startswith("model/")
            self._serve_file(STATIC_DIR / rel, cache=immutable_asset)
            return
        if path == "/favicon.ico":
            self._send_bytes(HTTPStatus.NO_CONTENT, b"", "image/x-icon")
            return
        if path == "/api/latest":
            self._send_json(HTTPStatus.OK, state.envelope())
            return
        if path == "/api/pose":
            self._send_json(HTTPStatus.OK, state.pose_envelope())
            return
        if path == "/api/events":
            env = state.envelope()
            self._send_json(HTTPStatus.OK, {"bridge": env["bridge"], "events": env["events"]})
            return
        if path == "/api/system":
            self._send_json(HTTPStatus.OK, system_state.envelope())
            return
        if path == "/api/unity/telemetry":
            self._send_json(
                HTTPStatus.OK,
                unity_telemetry_snapshot(
                    state,
                    system_state,
                ),
            )
            return
        if path == "/api/controller":
            manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]
            self._send_json(HTTPStatus.OK, manager.status())
            return
        if path == "/api/controller/config":
            manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]
            self._send_json(HTTPStatus.OK, manager.config_schema())
            return
        if path == "/api/controller/log":
            manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]
            query = parse_qs(parsed.query)
            try:
                lines = int((query.get("lines") or ["80"])[0])
            except ValueError:
                lines = 80
            self._send_json(HTTPStatus.OK, manager.log_tail(lines))
            return
        if path == "/api/camera":
            manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]
            self._send_json(HTTPStatus.OK, manager.camera_status())
            return
        if path == "/api/camera/pointcloud":
            manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]
            packet = manager.camera_pointcloud_snapshot()
            if packet is None:
                self._send_bytes(HTTPStatus.NO_CONTENT, b"", "application/octet-stream", cache=False)
            else:
                self._send_bytes(HTTPStatus.OK, packet, "application/vnd.g1.pointcloud", cache=False)
            return
        if path == "/api/camera/log":
            manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]
            query = parse_qs(parsed.query)
            try:
                lines = int((query.get("lines") or ["80"])[0])
            except ValueError:
                lines = 80
            self._send_json(HTTPStatus.OK, manager.camera_log_tail(lines))
            return
        if path == "/api/slam/status":
            self._send_json(HTTPStatus.OK, slam_status_snapshot())
            return
        if path == "/api/slam/map":
            try:
                if not SLAM_MAP_PATH.is_file():
                    raise FileNotFoundError(str(SLAM_MAP_PATH))
                body = SLAM_MAP_PATH.read_bytes()
            except OSError as exc:
                self._send_json(
                    HTTPStatus.NOT_FOUND,
                    {"error": f"SLAM map unavailable: {exc}"},
                )
                return
            self._send_bytes(
                HTTPStatus.OK,
                body,
                "application/vnd.pointcloud",
                cache=False,
            )
            return
        if path == "/api/slam/cloud":
            status = slam_status_snapshot()
            cloud = status.get("cloud")
            if (
                not status.get("worker_online")
                or not isinstance(cloud, dict)
                or not cloud.get("online")
            ):
                self._send_bytes(
                    HTTPStatus.NO_CONTENT,
                    b"",
                    "application/octet-stream",
                    cache=False,
                )
                return
            try:
                packet = SLAM_CLOUD_PATH.read_bytes()
                if not packet or len(packet) % 12:
                    raise ValueError(
                        "SLAM cloud must contain packed float32 XYZ triples"
                    )
                if len(packet) > 12 * 50000:
                    raise ValueError("SLAM cloud exceeds the configured limit")
            except (OSError, ValueError) as exc:
                self._send_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"error": f"SLAM cloud unavailable: {exc}"},
                )
                return
            self._send_bytes(
                HTTPStatus.OK,
                packet,
                "application/vnd.g1.slam-cloud-f32",
                cache=False,
            )
            return
        if path == "/api/services/control":
            client: ServiceActionClient = self.server.service_action_client  # type: ignore[attr-defined]
            policy = load_policy()
            worker = client.ping() if self.server.service_actions_enabled else {"status": "DISABLED", "reason": "service actions disabled"}  # type: ignore[attr-defined]
            self._send_json(HTTPStatus.OK, {
                "schema": "g1_dashboard.service_control.v1",
                "enabled": bool(self.server.service_actions_enabled),  # type: ignore[attr-defined]
                "management_key_required": bool(self.server.service_actions_enabled),  # type: ignore[attr-defined]
                "policy": public_policy(policy),
                "worker": worker,
            })
            return
        if path == "/api/info":
            env = state.envelope()
            self._send_json(HTTPStatus.OK, {
                "bridge": env["bridge"],
                "safety_boundary": {
                    "bridge_imports_robot_dds": False,
                    "browser_direct_robot_dds_commands": False,
                    "service_switch_worker_isolated": True,
                    "controller_xr_action_endpoint": bool(self.server.process_actions_enabled),
                    "controller_xr_action_controller_validated": True,
                    "controller_process_actions": bool(self.server.process_actions_enabled),  # type: ignore[attr-defined]
                    "controller_process_actions_authenticated": True,
                    "inspire_dependency_lifecycle": "root-owned fixed helper; controller first on stop",
                    "camera_process_actions": bool(self.server.process_actions_enabled),
                    "camera_process_actions_authenticated": True,
                    "camera_display_modes": ["rgb", "depth", "overlay", "near", "disparity", "pointcloud", "topdown"],
                    "camera_mode_switch_preserves_webrtc": True,
                    "camera_yolo_live_toggle": True,
                    "camera_yolo_inference_source": "single aligned RGB frame",
                    "camera_point_view_orbit_control": True,
                    "camera_pointcloud_browser_webgl": True,
                    "camera_pointcloud_transport": "latest-only binary G1PC over same-origin HTTP",
                    "camera_topdown_projection": "orthographic X/Z occupancy grid",
                    "slam_worker_isolated": True,
                    "slam_worker_control": (
                        "authenticated local-file relay; API 1804 only"
                    ),
                    "slam_initial_pose_endpoint": "/api/slam/initialize",
                    "slam_saved_map_endpoint": "/api/slam/map",
                    "slam_live_cloud_endpoint": "/api/slam/cloud",
                    "slam_status_endpoint": "/api/slam/status",
                    "slam_map": str(SLAM_MAP_PATH),
                    "unitree_service_actions": bool(self.server.service_actions_enabled),  # type: ignore[attr-defined]
                    "unitree_service_actions_authenticated": True,
                    "unitree_service_actions_allowlisted": True,
                    "unitree_service_actions_post_verified": True,
                    "bridge_imports_unitree_dds": False,
                    "arbitrary_process_launch": False,
                    "telemetry_input": f"udp://127.0.0.1:{self.server.udp_port}",  # type: ignore[attr-defined]
                    "system_input": f"udp://127.0.0.1:{self.server.system_udp_port}",  # type: ignore[attr-defined]
                },
            })
            return
        if path == "/healthz":
            env = state.envelope()
            self._send_json(HTTPStatus.OK if env["bridge"]["telemetry_online"] else HTTPStatus.SERVICE_UNAVAILABLE,
                            {"ok": env["bridge"]["telemetry_online"], "bridge": env["bridge"]})
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]

        if path not in (
            "/api/controller/auth",
            "/api/controller/start",
            "/api/controller/stop",
            "/api/controller/action",
            "/api/camera/start",
            "/api/camera/stop",
            "/api/camera/mode",
            "/api/camera/yolo",
            "/api/camera/view",
            "/api/services/set",
            "/api/slam/initialize",
        ):
            self._send_json(HTTPStatus.METHOD_NOT_ALLOWED, {
                "error": "unsupported POST; authenticated endpoints are controller auth/start/stop/action, camera start/stop/mode/yolo/view, and allowlisted service set"
            })
            return
        if not self._require_process_action_auth():
            return
        try:
            payload = self._read_json_body()
            if not isinstance(payload, dict):
                raise ValueError("JSON body must be an object")
            if path == "/api/controller/auth":
                self._send_json(HTTPStatus.OK, {
                    "ok": True,
                    "authenticated": True,
                    "controller": manager.status(),
                })
            elif path == "/api/controller/start":
                status = manager.start(payload.get("parameters", {}))
                self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "controller": status})
            elif path == "/api/controller/stop":
                status = manager.stop()
                self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "controller": status})
            elif path == "/api/controller/action":
                response = manager.request_xr_action(payload.get("operation", ""))
                accepted = response.get("status") == "ACCEPTED"
                self._send_json(
                    HTTPStatus.ACCEPTED if accepted else HTTPStatus.CONFLICT,
                    {"ok": accepted, "action": response, "controller": manager.status()},
                )
            elif path == "/api/slam/initialize":
                request = write_slam_initialize_request(payload)
                self._send_json(
                    HTTPStatus.ACCEPTED,
                    {
                        "ok": True,
                        "initialization": request,
                    },
                )
            elif path == "/api/services/set":
                if not self.server.service_actions_enabled:  # type: ignore[attr-defined]
                    raise PermissionError("Unitree service actions are disabled")
                service = str(payload.get("service") or "").strip()
                enabled = payload.get("enabled")
                if not service or len(service) > 128:
                    raise ValueError("service must be a non-empty service name")
                if not isinstance(enabled, bool):
                    raise ValueError("enabled must be boolean")
                policy = load_policy()
                bridge_policy = classify_service(service, unitree_protect=False, policy=policy)
                if bridge_policy != "ALLOWED":
                    raise PermissionError(f"service {service!r} policy is {bridge_policy}; explicit ALLOWED policy is required")
                client: ServiceActionClient = self.server.service_action_client  # type: ignore[attr-defined]
                response = client.request("SET_SERVICE", service=service, enabled=enabled)
                completed = response.get("status") == "COMPLETED" and response.get("verified") is True
                self._send_json(
                    HTTPStatus.OK if completed else HTTPStatus.CONFLICT,
                    {"ok": completed, "service_action": response},
                )
            elif path == "/api/camera/start":
                status = manager.start_camera()
                self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "camera": status})
            elif path == "/api/camera/stop":
                status = manager.stop_camera()
                self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "camera": status})
            elif path == "/api/camera/mode":
                mode = str(payload.get("mode") or "").strip().lower()
                status = manager.set_camera_mode(mode)
                acknowledged = bool(status.get("mode_ack_online") and status.get("mode_actual") == mode)
                self._send_json(
                    HTTPStatus.OK if acknowledged else HTTPStatus.ACCEPTED,
                    {"ok": True, "acknowledged": acknowledged, "camera": status},
                )
            elif path == "/api/camera/yolo":
                enabled = payload.get("enabled")
                if not isinstance(enabled, bool):
                    raise ValueError("enabled must be a boolean")
                status = manager.set_camera_yolo(enabled)
                acknowledged = bool(
                    status.get("yolo_ack_online")
                    and status.get("yolo_actual") == enabled
                )
                self._send_json(
                    HTTPStatus.OK if acknowledged else HTTPStatus.ACCEPTED,
                    {
                        "ok": True,
                        "acknowledged": acknowledged,
                        "camera": status,
                    },
                )
            elif path == "/api/camera/view":
                view = payload.get("view")
                if not isinstance(view, dict):
                    raise ValueError("view must be an object")
                status = manager.set_camera_point_view(view)
                self._send_json(HTTPStatus.OK, {"ok": True, "camera": status})
        except PermissionError as exc:
            self._send_json(HTTPStatus.FORBIDDEN, {"error": str(exc), "controller": manager.status(), "camera": manager.camera_status()})
        except FileNotFoundError as exc:
            self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc), "controller": manager.status(), "camera": manager.camera_status()})
        except (ValueError, RuntimeError) as exc:
            self._send_json(HTTPStatus.CONFLICT if isinstance(exc, RuntimeError) else HTTPStatus.BAD_REQUEST, {
                "error": str(exc), "controller": manager.status(), "camera": manager.camera_status()
            })
        except Exception as exc:
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": f"dashboard action failed: {exc}"})


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Dependency-free read-only G1 dashboard telemetry bridge")
    p.add_argument("--udp-host", default="127.0.0.1")
    p.add_argument("--udp-port", type=int, default=8765)
    p.add_argument("--system-udp-port", type=int, default=8766)
    p.add_argument("--robot-udp-port", type=int, default=8768)
    p.add_argument("--http-host", default="0.0.0.0")
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--stale-after-s", type=float, default=1.5)
    p.add_argument(
        "--enable-process-actions",
        action="store_true",
        help="Enable authenticated controller/Inspire/camera lifecycle actions and Step 5.1 XR action forwarding.",
    )
    p.add_argument(
        "--enable-service-actions",
        action="store_true",
        help="Enable authenticated, explicitly allowlisted Unitree RobotState service actions through the separate g1_xr worker.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.udp_host not in ("127.0.0.1", "localhost"):
        raise SystemExit("Refusing non-loopback telemetry input. Keep --udp-host=127.0.0.1.")
    if not (
        1 <= args.udp_port <= 65535
        and 1 <= args.system_udp_port <= 65535
        and 1 <= args.robot_udp_port <= 65535
        and 1 <= args.http_port <= 65535
    ):
        raise SystemExit(
            "Ports must be in [1, 65535]."
        )
    if args.stale_after_s <= 0:
        raise SystemExit("--stale-after-s must be positive.")

    state = BridgeState(stale_after_s=args.stale_after_s)
    udp_thread = UdpTelemetryThread(
        state,
        "127.0.0.1",
        args.udp_port,
    )
    udp_thread.start()

    robot_thread = UdpRobotTelemetryThread(
        state,
        "127.0.0.1",
        args.robot_udp_port,
    )
    robot_thread.start()

    system_state = SystemState(stale_after_s=3.0)
    system_thread = UdpSystemThread(system_state, "127.0.0.1", args.system_udp_port)
    system_thread.start()

    server = ThreadingHTTPServer((args.http_host, args.http_port), DashboardHandler)
    server.daemon_threads = True
    server.state = state  # type: ignore[attr-defined]
    server.system_state = system_state  # type: ignore[attr-defined]
    server.udp_port = args.udp_port  # type: ignore[attr-defined]
    server.system_udp_port = args.system_udp_port  # type: ignore[attr-defined]
    server.robot_udp_port = args.robot_udp_port  # type: ignore[attr-defined]
    process_manager = ControllerProcessManager(enabled=args.enable_process_actions)
    action_token = os.environ.get("G1_DASHBOARD_ACTION_TOKEN", "") if args.enable_process_actions else ""
    if args.enable_process_actions and len(action_token) < 16:
        raise SystemExit("--enable-process-actions requires G1_DASHBOARD_ACTION_TOKEN with at least 16 characters")

    fullbody_status = process_manager.ensure_dashboard_fullbody_sender()
    server.process_manager = process_manager  # type: ignore[attr-defined]
    server.process_actions_enabled = args.enable_process_actions  # type: ignore[attr-defined]
    server.action_token = action_token  # type: ignore[attr-defined]
    service_action_client = ServiceActionClient()
    if args.enable_service_actions and not service_action_client.configured():
        raise SystemExit("--enable-service-actions requires G1_DASHBOARD_SERVICE_SOCKET and G1_DASHBOARD_SERVICE_TOKEN")
    if args.enable_service_actions and not args.enable_process_actions:
        raise SystemExit("--enable-service-actions requires --enable-process-actions so management-key auth is available")
    server.service_action_client = service_action_client  # type: ignore[attr-defined]
    server.service_actions_enabled = args.enable_service_actions  # type: ignore[attr-defined]

    print(f"{BRIDGE_VERSION} PROCESS+DEPENDENCY+XR+SERVICE-ACTION-CONTROL")
    print(f"Telemetry input : udp://127.0.0.1:{args.udp_port}")
    print(f"Robot input     : udp://127.0.0.1:{args.robot_udp_port}")
    print(f"System input    : udp://127.0.0.1:{args.system_udp_port}")
    print(f"Dashboard HTTP  : http://{args.http_host}:{args.http_port}")
    print("Transport       : dual-rate latest-only HTTP (30 Hz pose / 4 Hz status)")
    print("Dependencies    : Python standard library only")
    print(f"Process actions : {'ENABLED' if args.enable_process_actions else 'DISABLED'}")
    print(
        "Quest full body: "
        f"{fullbody_status.get('state')} "
        f"pid={fullbody_status.get('pid')}"
    )
    print(f"Service actions : {'ENABLED (explicit allowlist + post-verification)' if args.enable_service_actions else 'DISABLED'}")
    if args.enable_process_actions:
        print(f"Controller      : {process_manager.script_path}")
        print(f"Controller Py   : {process_manager.python_path}")
        print("Auth            : X-G1-Management-Key required")
    print("Bridge imports no DDS. XR requests are controller-validated; Unitree ServiceSwitch is isolated in the allowlisted service worker and post-verified with ServiceList.")
    interrupted = False
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        interrupted = True
        print("\nStopping dashboard bridge...")
    finally:
        # A direct Ctrl+C of the bridge is an explicit lifecycle command. An
        # unexpected bridge exception remains non-authoritative and does not
        # automatically change robot-side process state.
        if interrupted:
            try:
                if args.enable_process_actions:
                    summary = process_manager.shutdown_dashboard_managed(
                        timeout_s=15.0
                    )
                else:
                    fullbody = process_manager.fullbody_sender.stop(
                        timeout_s=3.0
                    )
                    summary = {
                        "fullbody": fullbody.get("state")
                    }

                print(f"Managed dependency shutdown: {summary}")
            except Exception as exc:
                print(
                    "WARNING: managed dependency shutdown failed: "
                    f"{exc}"
                )
        # serve_forever() has already returned here; calling shutdown() from the
        # same thread can deadlock. Close the socket directly.
        server.server_close()
        udp_thread.stop()
        robot_thread.stop()
        system_thread.stop()
        udp_thread.join(timeout=1.0)
        robot_thread.join(timeout=1.0)
        system_thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
