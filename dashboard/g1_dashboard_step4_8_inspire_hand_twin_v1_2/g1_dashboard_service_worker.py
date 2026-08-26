#!/usr/bin/env python3
"""Controller-independent, allowlisted Unitree RobotState service action worker.

The browser never talks to this process directly. The stdlib dashboard bridge
validates the management key, then sends one whitelisted request over a 0600
Unix-domain socket. This worker is the only dashboard component that imports the
Unitree SDK and the only one that may call RobotStateClient.ServiceSwitch().
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import signal
import socket
import threading
import time
from pathlib import Path
from typing import Any

from g1_dashboard_service_policy import classify_service, load_policy, public_policy

SCHEMA = "g1_dashboard.service_action.v1"
VERSION = "g1_dashboard_service_worker.v1.0-verified-switch"
STOP = threading.Event()
MAX_REQUEST_BYTES = 64 * 1024


class ServiceWorker:
    CANDIDATES = (
        "unitree_sdk2py.g1.robot_state.robot_state_client",
        "unitree_sdk2py.go2.robot_state.robot_state_client",
        "unitree_sdk2py.b2.robot_state.robot_state_client",
    )

    def __init__(self, interface: str, token: str, policy_path: str | None = None) -> None:
        self.interface = interface
        self.token = token
        self.policy_path = policy_path
        self.policy = load_policy(policy_path)
        self.client: Any = None
        self.module: str | None = None
        self._lock = threading.Lock()

    @staticmethod
    def _enabled_from_status(status: Any) -> bool | None:
        try:
            value = int(status)
        except Exception:
            return None
        # Unitree RobotState ServiceList polarity observed by the existing
        # monitor: 0 = ON, 1 = OFF.
        return True if value == 0 else False if value == 1 else None

    def initialize(self) -> None:
        channel = importlib.import_module("unitree_sdk2py.core.channel")
        channel.ChannelFactoryInitialize(0, self.interface)
        errors: list[str] = []
        cls = None
        module = None
        for name in self.CANDIDATES:
            try:
                mod = importlib.import_module(name)
                cls = getattr(mod, "RobotStateClient")
                module = name
                break
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if cls is None:
            raise RuntimeError("RobotStateClient import failed: " + " | ".join(errors))
        client = cls()
        if hasattr(client, "SetTimeout"):
            client.SetTimeout(2.0)
        client.Init()
        self.client = client
        self.module = module
        # Validate read connectivity before accepting a socket connection.
        self._service_map()

    def _service_map(self) -> dict[str, dict[str, Any]]:
        if self.client is None:
            raise RuntimeError("RobotStateClient is not initialized")
        code, services = self.client.ServiceList()
        if int(code) != 0:
            raise RuntimeError(f"ServiceList returned code {code}")
        out: dict[str, dict[str, Any]] = {}
        for svc in services or []:
            name = str(getattr(svc, "name", "?") or "?")
            raw_status = int(getattr(svc, "status", -1))
            protect = bool(getattr(svc, "protect", False))
            out[name] = {
                "name": name,
                "status": raw_status,
                "enabled": self._enabled_from_status(raw_status),
                "protect": protect,
                "policy": classify_service(name, unitree_protect=protect, policy=self.policy),
            }
        return out

    def _base_response(self, req: dict[str, Any]) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "worker_version": VERSION,
            "request_id": str(req.get("request_id") or ""),
            "unix_time_s": time.time(),
        }

    def handle(self, req: dict[str, Any]) -> dict[str, Any]:
        base = self._base_response(req)
        # Reload the small policy file on every request so bridge/worker policy
        # cannot drift if an operator deliberately updates the allowlist while
        # the dashboard stack is running.
        self.policy = load_policy(self.policy_path)
        if req.get("schema") != SCHEMA:
            return {**base, "status": "REJECTED", "reason": "invalid schema"}
        if str(req.get("token") or "") != self.token:
            return {**base, "status": "REJECTED", "reason": "invalid internal token"}
        operation = str(req.get("operation") or "")
        if operation == "PING":
            return {
                **base,
                "status": "READY",
                "reason": "service action worker ready",
                "module": self.module,
                "policy": public_policy(self.policy),
            }
        if operation != "SET_SERVICE":
            return {**base, "status": "REJECTED", "reason": "unsupported operation"}

        name = str(req.get("service") or "").strip()
        desired = req.get("enabled")
        if not name or len(name) > 128 or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-." for ch in name):
            return {**base, "status": "REJECTED", "reason": "invalid service name"}
        if not isinstance(desired, bool):
            return {**base, "status": "REJECTED", "reason": "enabled must be boolean"}

        with self._lock:
            try:
                before_map = self._service_map()
                before = before_map.get(name)
                if before is None:
                    return {**base, "status": "REJECTED", "reason": f"service {name!r} is not present"}
                policy = before["policy"]
                if policy != "ALLOWED":
                    return {
                        **base,
                        "status": "REJECTED",
                        "reason": f"service policy is {policy}; only explicitly ALLOWED services can be switched",
                        "service": name,
                        "before": before,
                    }
                if before.get("enabled") is desired:
                    return {
                        **base,
                        "status": "COMPLETED",
                        "reason": "service already in requested state",
                        "service": name,
                        "requested_enabled": desired,
                        "before": before,
                        "after": before,
                        "verified": True,
                        "changed": False,
                    }

                # Official unitree_sdk2_python ServiceSwitch(name, switch: bool)
                # serializes bool as int(switch), so True requests ON and False OFF.
                code = int(self.client.ServiceSwitch(name, bool(desired)))
                if code != 0:
                    return {
                        **base,
                        "status": "FAILED",
                        "reason": f"ServiceSwitch returned code {code}",
                        "service": name,
                        "requested_enabled": desired,
                        "before": before,
                        "service_switch_code": code,
                        "verified": False,
                    }

                after = None
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    time.sleep(0.20)
                    current = self._service_map().get(name)
                    if current is not None:
                        after = current
                        if current.get("enabled") is desired:
                            break
                verified = bool(after is not None and after.get("enabled") is desired)
                return {
                    **base,
                    "status": "COMPLETED" if verified else "FAILED",
                    "reason": "post-switch ServiceList verification succeeded" if verified else "ServiceSwitch returned success but ServiceList did not confirm the requested state",
                    "service": name,
                    "requested_enabled": desired,
                    "before": before,
                    "after": after,
                    "service_switch_code": code,
                    "verified": verified,
                    "changed": verified,
                }
            except Exception as exc:
                return {
                    **base,
                    "status": "FAILED",
                    "reason": f"{type(exc).__name__}: {exc}",
                    "service": name,
                    "requested_enabled": desired,
                    "verified": False,
                }


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Allowlisted Unitree RobotState service action worker")
    p.add_argument("--network-interface", default="enP8p1s0")
    p.add_argument("--socket", required=True)
    p.add_argument("--policy", default=None)
    return p.parse_args()


def _read_request(conn: socket.socket) -> dict[str, Any]:
    chunks: list[bytes] = []
    total = 0
    while total < MAX_REQUEST_BYTES:
        chunk = conn.recv(min(65536, MAX_REQUEST_BYTES - total))
        if not chunk:
            break
        chunks.append(chunk)
        total += len(chunk)
        if b"\n" in chunk:
            break
    raw = b"".join(chunks).split(b"\n", 1)[0].decode("utf-8", "replace")
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("request must be a JSON object")
    return value


def main() -> int:
    args = parse_args()
    token = os.environ.get("G1_DASHBOARD_SERVICE_TOKEN", "")
    if len(token) < 16:
        raise SystemExit("G1_DASHBOARD_SERVICE_TOKEN must contain at least 16 characters")
    path = Path(args.socket)
    if not path.is_absolute():
        raise SystemExit("--socket must be an absolute path")
    try:
        path.unlink(missing_ok=True)
    except Exception as exc:
        raise SystemExit(f"cannot remove stale service socket {path}: {exc}")

    worker = ServiceWorker(args.network_interface, token, args.policy)
    worker.initialize()

    def stop_handler(signum: int, frame: object) -> None:
        del signum, frame
        STOP.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(path))
    os.chmod(path, 0o600)
    server.listen(4)
    server.settimeout(0.25)
    print(f"{VERSION}", flush=True)
    print(f"Service socket    : {path} (0600)", flush=True)
    print(f"Network interface : {args.network_interface}", flush=True)
    print(f"RobotState module : {worker.module}", flush=True)
    print(f"Allowed services  : {', '.join(worker.policy.get('allowed_services', [])) or '(none)'}", flush=True)
    print("Unitree protect bit + hard deny + explicit allowlist are enforced.", flush=True)
    try:
        while not STOP.is_set():
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            except OSError:
                if STOP.is_set():
                    break
                raise
            with conn:
                conn.settimeout(5.0)
                try:
                    req = _read_request(conn)
                    response = worker.handle(req)
                except Exception as exc:
                    response = {
                        "schema": SCHEMA,
                        "worker_version": VERSION,
                        "status": "REJECTED",
                        "reason": f"invalid request: {type(exc).__name__}: {exc}",
                        "unix_time_s": time.time(),
                    }
                conn.sendall((json.dumps(response, separators=(",", ":")) + "\n").encode("utf-8"))
    finally:
        try:
            server.close()
        except Exception:
            pass
        try:
            path.unlink(missing_ok=True)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
