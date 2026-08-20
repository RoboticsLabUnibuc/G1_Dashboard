#!/usr/bin/env python3
"""G1 dashboard bridge v1.6.0 — RealSense camera display modes + verified service actions.

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
BRIDGE_VERSION = "g1_dashboard_bridge.v1.6.0-realsense-camera-modes"
SYSTEM_SCHEMA = "g1_dashboard.system.v1"
STATIC_DIR = Path(__file__).resolve().parent / "static"


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
            stale = age is None or age > self.stale_after_s
            if stale and not self._stale_latched:
                self._stale_latched = True
                self.add_event("warning", "bridge", f"Telemetry link stale (> {self.stale_after_s:.1f}s without packets)")

    def bridge_meta_locked(self) -> dict[str, Any]:
        age = self._packet_age_locked()
        return {
            "version": BRIDGE_VERSION,
            "schema": SCHEMA,
            "transport": "http-polling",
            "telemetry_online": age is not None and age <= self.stale_after_s,
            "packet_age_s": finite_number(age),
            "packet_count": self.packet_count,
            "invalid_count": self.invalid_count,
            "last_source": self.last_source,
            "stale_after_s": self.stale_after_s,
        }

    def envelope(self) -> dict[str, Any]:
        with self._lock:
            self.update_stale_event()
            return {
                "type": "telemetry",
                "bridge": self.bridge_meta_locked(),
                "telemetry": self.latest,
                "events": list(self.events)[:100],
            }

    def pose_envelope(self) -> dict[str, Any]:
        """Small latest-only pose payload for the browser's 30 Hz render path."""
        with self._lock:
            age = self._packet_age_locked()
            t = self.latest
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
                "sequence": t.get("sequence"),
                "source_unix_time_s": t.get("unix_time_s"),
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
        if self.path.startswith("/api/pose"):
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
        if path == "/api/camera/log":
            manager: ControllerProcessManager = self.server.process_manager  # type: ignore[attr-defined]
            query = parse_qs(parsed.query)
            try:
                lines = int((query.get("lines") or ["80"])[0])
            except ValueError:
                lines = 80
            self._send_json(HTTPStatus.OK, manager.camera_log_tail(lines))
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
                    "camera_display_modes": ["rgb", "depth", "overlay", "near"],
                    "camera_mode_switch_preserves_webrtc": True,
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
            "/api/services/set",
        ):
            self._send_json(HTTPStatus.METHOD_NOT_ALLOWED, {
                "error": "unsupported POST; authenticated endpoints are controller auth/start/stop/action, camera start/stop/mode, and allowlisted service set"
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
    if not (1 <= args.udp_port <= 65535 and 1 <= args.system_udp_port <= 65535 and 1 <= args.http_port <= 65535):
        raise SystemExit("Ports must be in [1, 65535].")
    if args.stale_after_s <= 0:
        raise SystemExit("--stale-after-s must be positive.")

    state = BridgeState(stale_after_s=args.stale_after_s)
    udp_thread = UdpTelemetryThread(state, "127.0.0.1", args.udp_port)
    udp_thread.start()
    system_state = SystemState(stale_after_s=3.0)
    system_thread = UdpSystemThread(system_state, "127.0.0.1", args.system_udp_port)
    system_thread.start()

    server = ThreadingHTTPServer((args.http_host, args.http_port), DashboardHandler)
    server.daemon_threads = True
    server.state = state  # type: ignore[attr-defined]
    server.system_state = system_state  # type: ignore[attr-defined]
    server.udp_port = args.udp_port  # type: ignore[attr-defined]
    server.system_udp_port = args.system_udp_port  # type: ignore[attr-defined]
    process_manager = ControllerProcessManager(enabled=args.enable_process_actions)
    action_token = os.environ.get("G1_DASHBOARD_ACTION_TOKEN", "") if args.enable_process_actions else ""
    if args.enable_process_actions and len(action_token) < 16:
        raise SystemExit("--enable-process-actions requires G1_DASHBOARD_ACTION_TOKEN with at least 16 characters")
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
    print(f"System input    : udp://127.0.0.1:{args.system_udp_port}")
    print(f"Dashboard HTTP  : http://{args.http_host}:{args.http_port}")
    print("Transport       : dual-rate latest-only HTTP (30 Hz pose / 4 Hz status)")
    print("Dependencies    : Python standard library only")
    print(f"Process actions : {'ENABLED' if args.enable_process_actions else 'DISABLED'}")
    print(f"Service actions : {'ENABLED (explicit allowlist + post-verification)' if args.enable_service_actions else 'DISABLED'}")
    if args.enable_process_actions:
        print(f"Controller      : {process_manager.script_path}")
        print(f"Controller Py   : {process_manager.python_path}")
        print("Auth            : X-G1-Management-Key required")
    print("Bridge imports no DDS. XR requests are controller-validated; Unitree ServiceSwitch is isolated in the allowlisted service worker and post-verified with ServiceList.")
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nStopping dashboard bridge...")
    finally:
        # serve_forever() has already returned here; calling shutdown() from the
        # same thread can deadlock. Close the socket directly.
        server.server_close()
        udp_thread.stop()
        system_thread.stop()
        udp_thread.join(timeout=1.0)
        system_thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
