#!/usr/bin/env python3
"""Shared, stdlib-only policy helpers for dashboard Unitree service actions."""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

SCHEMA = "g1_dashboard.service_policy.v1"
DEFAULT_POLICY_PATH = Path(__file__).resolve().parent / "service_policy.json"
VALID_POLICIES = {"ALLOWED", "PROTECTED", "READ_ONLY", "UNKNOWN"}


def _names(values: Any) -> list[str]:
    if not isinstance(values, list):
        return []
    out: list[str] = []
    seen: set[str] = set()
    for value in values:
        name = str(value or "").strip()
        if name and name not in seen:
            out.append(name)
            seen.add(name)
    return out


def load_policy(path: str | Path | None = None) -> dict[str, Any]:
    policy_path = Path(path or os.environ.get("G1_DASHBOARD_SERVICE_POLICY", "") or DEFAULT_POLICY_PATH)
    raw: dict[str, Any] = {}
    try:
        loaded = json.loads(policy_path.read_text())
        if isinstance(loaded, dict):
            raw = loaded
    except FileNotFoundError:
        pass

    if raw and raw.get("schema") not in (None, SCHEMA):
        raise ValueError(f"Unexpected service policy schema: {raw.get('schema')!r}")

    allowed = _names(raw.get("allowed_services", []))
    readonly = _names(raw.get("read_only_services", []))
    hard_deny = _names(raw.get("hard_deny_services", []))

    # Runtime allowlist is intentionally additive and explicit. It is useful for
    # testing one reviewed service without editing the repository. It can never
    # override Unitree's protect bit, hard_deny, or read_only policy.
    env_allowed = [x.strip() for x in os.environ.get("G1_DASHBOARD_SERVICE_ALLOWLIST", "").split(",") if x.strip()]
    for name in env_allowed:
        if name not in allowed:
            allowed.append(name)

    allowed = [name for name in allowed if name not in hard_deny and name not in readonly]
    return {
        "schema": SCHEMA,
        "version": str(raw.get("version") or "step5.2-v1"),
        "path": str(policy_path),
        "default_policy": "UNKNOWN",
        "allowed_services": allowed,
        "read_only_services": readonly,
        "hard_deny_services": hard_deny,
        "runtime_allowlist": env_allowed,
    }


def classify_service(name: str, *, unitree_protect: bool = False, policy: dict[str, Any] | None = None) -> str:
    p = policy or load_policy()
    service = str(name or "").strip()
    if unitree_protect or service in set(p.get("hard_deny_services", [])):
        return "PROTECTED"
    if service in set(p.get("read_only_services", [])):
        return "READ_ONLY"
    if service in set(p.get("allowed_services", [])):
        return "ALLOWED"
    return "UNKNOWN"


def public_policy(policy: dict[str, Any] | None = None) -> dict[str, Any]:
    p = policy or load_policy()
    return {
        "schema": p["schema"],
        "version": p["version"],
        "default_policy": p["default_policy"],
        "allowed_services": list(p.get("allowed_services", [])),
        "read_only_services": list(p.get("read_only_services", [])),
        "hard_deny_services": list(p.get("hard_deny_services", [])),
        "runtime_allowlist": list(p.get("runtime_allowlist", [])),
    }
