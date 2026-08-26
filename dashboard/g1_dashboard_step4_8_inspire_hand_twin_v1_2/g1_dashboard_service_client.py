#!/usr/bin/env python3
"""Stdlib-only client for the localhost Unitree service action worker."""
from __future__ import annotations

import json
import os
import socket
import uuid
from pathlib import Path
from typing import Any

SCHEMA = "g1_dashboard.service_action.v1"


class ServiceActionClient:
    def __init__(self, *, socket_path: str | None = None, token: str | None = None, timeout_s: float = 5.0) -> None:
        self.socket_path = socket_path or os.environ.get("G1_DASHBOARD_SERVICE_SOCKET", "")
        self.token = token or os.environ.get("G1_DASHBOARD_SERVICE_TOKEN", "")
        self.timeout_s = float(timeout_s)

    def configured(self) -> bool:
        return bool(self.socket_path and len(self.token) >= 16)

    def socket_exists(self) -> bool:
        return bool(self.socket_path and Path(self.socket_path).exists())

    def request(self, operation: str, **payload: Any) -> dict[str, Any]:
        if not self.configured():
            raise RuntimeError("service action worker is not configured")
        req = {
            "schema": SCHEMA,
            "request_id": uuid.uuid4().hex,
            "token": self.token,
            "operation": str(operation),
            **payload,
        }
        data = (json.dumps(req, separators=(",", ":")) + "\n").encode("utf-8")
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        try:
            sock.connect(self.socket_path)
            sock.sendall(data)
            sock.shutdown(socket.SHUT_WR)
            chunks: list[bytes] = []
            total = 0
            while total < 1024 * 1024:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                chunks.append(chunk)
                total += len(chunk)
        finally:
            sock.close()
        raw = b"".join(chunks).decode("utf-8", "replace").strip()
        if not raw:
            raise RuntimeError("service action worker returned no response")
        response = json.loads(raw)
        if not isinstance(response, dict):
            raise RuntimeError("service action worker returned invalid JSON")
        return response

    def ping(self) -> dict[str, Any]:
        try:
            return self.request("PING")
        except Exception as exc:
            return {
                "schema": SCHEMA,
                "status": "OFFLINE",
                "reason": f"{type(exc).__name__}: {exc}",
            }
