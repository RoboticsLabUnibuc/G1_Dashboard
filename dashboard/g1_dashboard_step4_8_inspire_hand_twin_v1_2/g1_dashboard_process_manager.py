#!/usr/bin/env python3
"""Whitelisted lifecycle manager for the validated G1 XR controller.

This module intentionally does not import Unitree DDS libraries. It can only
start/stop one exact controller script and only with server-defined arguments.
The browser never supplies an executable path, shell fragment, environment
variable, or arbitrary command-line token.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import signal
import secrets
import socket
import uuid
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

CONTROLLER_BASENAME = (
    "g1_locomotion_xr_handover_live_v6_7_4_symmetric_thumb_control_"
    "dashboard_telemetry_v1_8.py"
)
MANAGER_SCHEMA = "g1_dashboard.controller_process.v1"
MANAGER_VERSION = "g1_dashboard_process_manager.v1.1-step5-1-xr-actions"
CONTROLLER_SHA256 = "66bb5c0bd99e1c034426c2395e46229a54d1f690fe607ed2f513c233e50fa758"

ACTION_REQUEST_SCHEMA = "g1_dashboard.action_request.v1"
ACTION_RESPONSE_SCHEMA = "g1_dashboard.action_response.v1"
ACTION_HOST = "127.0.0.1"
ACTION_PORT = 8767
ACTION_OPERATIONS = {"REQUEST_XR", "CANCEL_XR_REQUEST", "HAND_BACK_ARMS"}

# These defaults exactly match the last validated V1.8 launch command.
# Min/max values are dashboard edit bounds only. They are not robot safety
# limits and do not replace validation performed inside the controller.
PARAMETER_SPECS: list[dict[str, Any]] = [
    {"name":"frequency","flag":"--frequency","label":"Control rate","section":"Runtime","type":"float","default":30.0,"min":10.0,"max":60.0,"step":1.0,"unit":"Hz"},
    {"name":"status_hz","flag":"--status-hz","label":"Status rate","section":"Runtime","type":"float","default":4.0,"min":1.0,"max":10.0,"step":1.0,"unit":"Hz"},

    {"name":"max_wrist_speed","flag":"--max-wrist-speed","label":"Max wrist speed","section":"Arms","type":"float","default":0.18,"min":0.05,"max":0.30,"step":0.01,"unit":"m/s"},
    {"name":"max_wrist_rotation_speed_deg","flag":"--max-wrist-rotation-speed-deg","label":"Max wrist rotation","section":"Arms","type":"float","default":90.0,"min":30.0,"max":180.0,"step":5.0,"unit":"deg/s"},
    {"name":"joint_target_speed_rps","flag":"--joint-target-speed-rps","label":"Joint target speed","section":"Arms","type":"float","default":3.0,"min":0.5,"max":5.0,"step":0.1,"unit":"rad/s"},
    {"name":"max_ik_target_jump_rad","flag":"--max-ik-target-jump-rad","label":"Max IK target jump","section":"Arms","type":"float","default":0.25,"min":0.05,"max":0.50,"step":0.01,"unit":"rad"},
    {"name":"bimanual_inward_offset_m","flag":"--bimanual-inward-offset-m","label":"Bimanual inward offset","section":"Arms","type":"float","default":0.00,"min":0.00,"max":0.05,"step":0.005,"unit":"m"},

    {"name":"tracking_fault_frames","flag":"--tracking-fault-frames","label":"Tracking fault frames","section":"Tracking","type":"int","default":3,"min":1,"max":10,"step":1,"unit":"frames"},
    {"name":"tracking_resume_position_m","flag":"--tracking-resume-position-m","label":"Resume position tolerance","section":"Tracking","type":"float","default":0.07,"min":0.02,"max":0.15,"step":0.005,"unit":"m"},
    {"name":"tracking_resume_rotation_deg","flag":"--tracking-resume-rotation-deg","label":"Resume rotation tolerance","section":"Tracking","type":"float","default":25.0,"min":5.0,"max":60.0,"step":1.0,"unit":"deg"},
    {"name":"tracking_resume_stable_frames","flag":"--tracking-resume-stable-frames","label":"Resume stable frames","section":"Tracking","type":"int","default":8,"min":1,"max":30,"step":1,"unit":"frames"},

    {"name":"finger_frequency","flag":"--finger-frequency","label":"Finger command rate","section":"Hands","type":"float","default":90.0,"min":30.0,"max":150.0,"step":5.0,"unit":"Hz"},
    {"name":"finger_command_speed_per_s","flag":"--finger-command-speed-per-s","label":"Finger command speed","section":"Hands","type":"float","default":1.00,"min":0.20,"max":2.00,"step":0.05,"unit":"norm/s"},
    {"name":"finger_minimum_command","flag":"--finger-minimum-command","label":"Minimum finger command","section":"Hands","type":"float","default":0.10,"min":0.00,"max":0.40,"step":0.01,"unit":"norm"},
    {"name":"finger_stable_tracking_frames","flag":"--finger-stable-tracking-frames","label":"Finger stable tracking","section":"Hands","type":"int","default":15,"min":3,"max":40,"step":1,"unit":"frames"},
    {"name":"finger_tracking_fault_frames","flag":"--finger-tracking-fault-frames","label":"Finger fault frames","section":"Hands","type":"int","default":3,"min":1,"max":10,"step":1,"unit":"frames"},
    {"name":"finger_tracking_stale_s","flag":"--finger-tracking-stale-s","label":"Finger tracking stale","section":"Hands","type":"float","default":0.25,"min":0.10,"max":1.00,"step":0.05,"unit":"s"},
    {"name":"finger_reacquire_command_tolerance","flag":"--finger-reacquire-command-tolerance","label":"Finger reacquire tolerance","section":"Hands","type":"float","default":0.18,"min":0.05,"max":0.50,"step":0.01,"unit":"norm"},
    {"name":"finger_reacquire_stable_frames","flag":"--finger-reacquire-stable-frames","label":"Finger reacquire stable","section":"Hands","type":"int","default":6,"min":1,"max":20,"step":1,"unit":"frames"},
    {"name":"finger_state_stale_s","flag":"--finger-state-stale-s","label":"Finger state stale","section":"Hands","type":"float","default":0.50,"min":0.10,"max":2.00,"step":0.05,"unit":"s"},

    {"name":"allow_locomotion_during_xr","flag":"--allow-locomotion-during-xr","label":"Allow locomotion during XR","section":"Behavior","type":"bool","default":True},
]

FIXED_ARGS: list[str] = [
    "--enable-live-arm-sdk",
    "--network-interface=enP8p1s0",
    "--display-mode=immersive",
    "--img-server-ip=192.168.0.116",
    "--webrtc-port=60001",
    "--head-camera-height=480",
    "--head-camera-width=640",
    "--display-fps=30",
    "--teleop-weight=1.00",
    "--dashboard-telemetry",
    "--dashboard-telemetry-hz=30",
]

LOCKED_SETTINGS: list[dict[str, str]] = [
    {"label":"Controller","value":CONTROLLER_BASENAME},
    {"label":"DDS interface","value":"enP8p1s0"},
    {"label":"Display mode","value":"immersive"},
    {"label":"Camera","value":"192.168.0.116:60001 · 640×480 · 30 fps"},
    {"label":"Arm ownership","value":"1.00 (validated, locked)"},
    {"label":"Dashboard telemetry","value":"127.0.0.1:8765 · 30 Hz"},
]


def _finite(value: Any) -> float:
    x = float(value)
    if not math.isfinite(x):
        raise ValueError("value must be finite")
    return x


def _format_number(value: float | int) -> str:
    if isinstance(value, int):
        return str(value)
    return format(float(value), ".8g")


def _proc_start_ticks(pid: int) -> int | None:
    try:
        # /proc/<pid>/stat field 22 is process start time in clock ticks.
        raw = Path(f"/proc/{pid}/stat").read_text()
        close = raw.rfind(")")
        fields = raw[close + 2:].split()
        return int(fields[19])
    except Exception:
        return None


def _proc_cmdline(pid: int) -> list[str]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        return [part.decode("utf-8", "replace") for part in raw.split(b"\0") if part]
    except Exception:
        return []


def _process_alive(pid: int, expected_start_ticks: int | None = None) -> bool:
    if pid <= 1 or not Path(f"/proc/{pid}").exists():
        return False
    if expected_start_ticks is not None and _proc_start_ticks(pid) != expected_start_ticks:
        return False
    return True


def _find_matching_pids(script_path: Path) -> list[int]:
    target = str(script_path.resolve())
    out: list[int] = []
    proc = Path("/proc")
    if not proc.exists():
        return out
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        args = _proc_cmdline(pid)
        if not args:
            continue
        for arg in args[1:]:
            try:
                # Manual launches commonly use only the basename from the teleop
                # directory. The exact basename is unique to this validated V1.8
                # controller, so treat it as a duplicate even if its cwd differs.
                if arg == target or Path(arg).name == CONTROLLER_BASENAME:
                    out.append(pid)
                    break
            except Exception:
                continue
    return sorted(set(out))


def default_controller_path() -> Path:
    override = os.environ.get("G1_DASHBOARD_CONTROLLER_SCRIPT", "").strip()
    path = Path(override).expanduser() if override else Path.home() / "xr_teleoperate_g1demo" / "teleop" / CONTROLLER_BASENAME
    if path.name != CONTROLLER_BASENAME:
        raise RuntimeError(
            f"G1_DASHBOARD_CONTROLLER_SCRIPT must point to {CONTROLLER_BASENAME}; got {path.name!r}"
        )
    return path


def default_controller_python() -> Path:
    override = os.environ.get("G1_DASHBOARD_CONTROLLER_PYTHON", "").strip()
    if override:
        return Path(override).expanduser()
    candidates = [
        Path.home() / "miniconda3" / "envs" / "g1_xr" / "bin" / "python",
        Path.home() / "miniforge3" / "envs" / "g1_xr" / "bin" / "python",
        Path.home() / "anaconda3" / "envs" / "g1_xr" / "bin" / "python",
        Path.home() / "miniconda3" / "envs" / "g1_deploy" / "bin" / "python",
        Path.home() / "miniforge3" / "envs" / "g1_deploy" / "bin" / "python",
        Path.home() / "anaconda3" / "envs" / "g1_deploy" / "bin" / "python",
        Path(sys.executable),
    ]
    for p in candidates:
        if p.is_file() and os.access(p, os.X_OK):
            return p
    return Path(sys.executable)


def _sha256_file(path: Path) -> str | None:
    try:
        h = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
        return h.hexdigest()
    except Exception:
        return None


def _controller_environment(python_path: Path) -> dict[str, str]:
    """Build a deterministic environment for the managed controller.

    The robot account has unrelated packages in ~/.local/lib/python3.10/site-packages.
    In particular, a package named ``pinocchio`` can shadow the validated conda
    Pinocchio build even when the g1_xr interpreter is invoked by absolute path.
    Managed launches therefore disable Python's user site and remove inherited
    Python path/home overrides.  We intentionally preserve LD_LIBRARY_PATH because
    the validated robot runtime relies on its existing native-library ordering.
    """
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONNOUSERSITE"] = "1"
    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)

    prefix = python_path.parent.parent
    env["PATH"] = f"{prefix / 'bin'}:{env.get('PATH', '')}"
    env["CONDA_PREFIX"] = str(prefix)
    env["CONDA_DEFAULT_ENV"] = prefix.name
    return env


def _controller_python_preflight(python_path: Path, env: dict[str, str], cwd: Path) -> str:
    """Verify the exact imports that previously failed before spawning V1.8."""
    code = (
        "import sys, numpy, pinocchio, cyclonedds; "
        "from pinocchio import casadi as cpin; "
        "print('python=' + sys.executable); "
        "print('numpy=' + str(getattr(numpy, '__version__', '?'))); "
        "print('pinocchio=' + str(getattr(pinocchio, '__version__', '?'))); "
        "print('pinocchio_file=' + str(getattr(pinocchio, '__file__', '?'))); "
        "print('pinocchio.casadi=OK'); "
        "print('cyclonedds=OK')"
    )
    try:
        result = subprocess.run(
            [str(python_path), "-c", code],
            cwd=str(cwd),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15.0,
            check=False,
            text=True,
        )
    except Exception as exc:
        raise RuntimeError(f"controller Python preflight could not run: {exc}") from exc

    output = (result.stdout or "").strip()
    if result.returncode != 0:
        raise RuntimeError(
            "controller Python preflight failed; managed launch refused. "
            f"Output: {output or '<no output>'}"
        )
    return output


class ControllerProcessManager:
    def __init__(self, *, enabled: bool) -> None:
        self.enabled = bool(enabled)
        self.script_path = default_controller_path()
        self.python_path = default_controller_python()
        self.state_path = Path(os.environ.get(
            "G1_DASHBOARD_CONTROLLER_STATE",
            "/tmp/g1_dashboard_controller_v1_8_state.json",
        ))
        self.log_path = Path(os.environ.get(
            "G1_DASHBOARD_CONTROLLER_LOG",
            str(Path.home() / ".local" / "state" / "g1_dashboard" / "controller_v1_8.log"),
        )).expanduser()
        self._lock = threading.RLock()
        self._popen: subprocess.Popen[bytes] | None = None
        self._managed: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._last_exit_code: int | None = None
        self._load_state()

    def _load_state(self) -> None:
        try:
            data = json.loads(self.state_path.read_text())
            pid = int(data.get("pid", 0))
            ticks = data.get("start_ticks")
            ticks = int(ticks) if ticks is not None else None
            if _process_alive(pid, ticks):
                args = _proc_cmdline(pid)
                if any(Path(a).name == CONTROLLER_BASENAME for a in args[1:]):
                    self._managed = data
                    return
        except Exception:
            pass
        self._managed = None
        try:
            self.state_path.unlink(missing_ok=True)
        except Exception:
            pass

    def _save_state(self) -> None:
        if self._managed is None:
            try:
                self.state_path.unlink(missing_ok=True)
            except Exception:
                pass
            return
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.state_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self._managed, indent=2, sort_keys=True))
            os.chmod(tmp, 0o600)
            tmp.replace(self.state_path)
        except Exception as exc:
            self._last_error = f"failed to persist process state: {exc}"

    def _validate_parameters(self, supplied: Any) -> dict[str, Any]:
        if supplied is None:
            supplied = {}
        if not isinstance(supplied, dict):
            raise ValueError("parameters must be an object")
        known = {spec["name"] for spec in PARAMETER_SPECS}
        unknown = sorted(set(supplied) - known)
        if unknown:
            raise ValueError(f"unknown parameter(s): {', '.join(unknown)}")
        out: dict[str, Any] = {}
        for spec in PARAMETER_SPECS:
            name = spec["name"]
            raw = supplied.get(name, spec["default"])
            typ = spec["type"]
            if typ == "bool":
                if not isinstance(raw, bool):
                    raise ValueError(f"{name} must be true/false")
                out[name] = raw
                continue
            if typ == "int":
                if isinstance(raw, bool):
                    raise ValueError(f"{name} must be an integer")
                x = _finite(raw)
                if not float(x).is_integer():
                    raise ValueError(f"{name} must be an integer")
                value: float | int = int(x)
            else:
                value = _finite(raw)
            lo, hi = float(spec["min"]), float(spec["max"])
            if float(value) < lo or float(value) > hi:
                raise ValueError(f"{name} must be between {spec['min']} and {spec['max']} {spec.get('unit','')}".strip())
            out[name] = value
        return out

    def build_command(self, parameters: Any) -> tuple[list[str], dict[str, Any]]:
        params = self._validate_parameters(parameters)
        cmd = [str(self.python_path), str(self.script_path), *FIXED_ARGS]
        by_name = {spec["name"]: spec for spec in PARAMETER_SPECS}
        for name, value in params.items():
            spec = by_name[name]
            if spec["type"] == "bool":
                if value:
                    cmd.append(spec["flag"])
            else:
                cmd.append(f"{spec['flag']}={_format_number(value)}")
        return cmd, params

    def config_schema(self) -> dict[str, Any]:
        return {
            "schema": MANAGER_SCHEMA,
            "manager_version": MANAGER_VERSION,
            "enabled": self.enabled,
            "controller_script": str(self.script_path),
            "controller_script_exists": self.script_path.is_file(),
            "controller_expected_sha256": CONTROLLER_SHA256,
            "controller_actual_sha256": _sha256_file(self.script_path) if self.script_path.is_file() else None,
            "controller_hash_match": _sha256_file(self.script_path) == CONTROLLER_SHA256 if self.script_path.is_file() else False,
            "controller_python": str(self.python_path),
            "controller_python_exists": self.python_path.is_file() and os.access(self.python_path, os.X_OK),
            "controller_user_site_disabled": True,
            "fixed_args": list(FIXED_ARGS),
            "locked_settings": list(LOCKED_SETTINGS),
            "parameter_specs": PARAMETER_SPECS,
            "known_good": {spec["name"]: spec["default"] for spec in PARAMETER_SPECS},
            "dashboard_edit_bounds_are_safety_limits": False,
            "management_key_required": True,
            "xr_action_channel": {
                "transport": f"udp://{ACTION_HOST}:{ACTION_PORT}",
                "loopback_only": True,
                "controller_validated": True,
                "operations": sorted(ACTION_OPERATIONS),
            },
            "log_path": str(self.log_path),
        }

    def _managed_alive(self) -> tuple[bool, int | None]:
        if self._managed is None:
            return False, None
        pid = int(self._managed.get("pid", 0))
        ticks = self._managed.get("start_ticks")
        ticks = int(ticks) if ticks is not None else None
        return _process_alive(pid, ticks), pid

    def _refresh(self) -> None:
        if self._popen is not None:
            rc = self._popen.poll()
            if rc is not None:
                self._last_exit_code = int(rc)
                self._popen = None
        alive, _pid = self._managed_alive()
        if self._managed is not None and not alive:
            self._managed = None
            self._save_state()

    def status(self) -> dict[str, Any]:
        with self._lock:
            self._refresh()
            matching = _find_matching_pids(self.script_path) if self.script_path.exists() else []
            managed_alive, managed_pid = self._managed_alive()
            external = [pid for pid in matching if pid != managed_pid]
            now = time.time()

            if managed_alive and self._managed is not None:
                stop_requested = self._managed.get("stop_requested_unix_time_s")
                state = "STOPPING" if stop_requested else "RUNNING"
                launched = float(self._managed.get("launched_unix_time_s", now))
                return {
                    "schema": MANAGER_SCHEMA,
                    "manager_version": MANAGER_VERSION,
                    "enabled": self.enabled,
                    "state": state,
                    "managed": True,
                    "pid": managed_pid,
                    "uptime_s": max(0.0, now - launched),
                    "launched_unix_time_s": launched,
                    "stop_requested_unix_time_s": stop_requested,
                    "parameters": self._managed.get("parameters", {}),
                    "command": self._managed.get("command", []),
                    "controller_script": str(self.script_path),
                    "controller_python": str(self.python_path),
                    "log_path": str(self.log_path),
                    "last_error": self._last_error,
                    "last_exit_code": self._last_exit_code,
                    "external_pids": external,
                    "can_start": False,
                    "can_stop": True,
                    "can_request_action": bool(self._managed.get("action_token")) and not bool(stop_requested),
                }

            if matching:
                return {
                    "schema": MANAGER_SCHEMA,
                    "manager_version": MANAGER_VERSION,
                    "enabled": self.enabled,
                    "state": "RUNNING_EXTERNAL",
                    "managed": False,
                    "pid": matching[0],
                    "uptime_s": None,
                    "parameters": None,
                    "command": None,
                    "controller_script": str(self.script_path),
                    "controller_python": str(self.python_path),
                    "log_path": str(self.log_path),
                    "last_error": self._last_error,
                    "last_exit_code": self._last_exit_code,
                    "external_pids": matching,
                    "can_start": False,
                    "can_stop": False,
                    "can_request_action": False,
                }

            script_hash = _sha256_file(self.script_path) if self.script_path.is_file() else None
            ready = (
                self.enabled
                and self.script_path.is_file()
                and script_hash == CONTROLLER_SHA256
                and self.python_path.is_file()
                and os.access(self.python_path, os.X_OK)
            )
            return {
                "schema": MANAGER_SCHEMA,
                "manager_version": MANAGER_VERSION,
                "enabled": self.enabled,
                "state": "STOPPED" if ready else "UNAVAILABLE",
                "managed": False,
                "pid": None,
                "uptime_s": None,
                "parameters": None,
                "command": None,
                "controller_script": str(self.script_path),
                "controller_python": str(self.python_path),
                "log_path": str(self.log_path),
                "last_error": self._last_error,
                "last_exit_code": self._last_exit_code,
                "external_pids": [],
                "can_start": ready,
                "can_stop": False,
                "can_request_action": False,
            }

    def start(self, parameters: Any) -> dict[str, Any]:
        with self._lock:
            if not self.enabled:
                raise PermissionError("controller process actions are disabled")
            self._refresh()
            if not self.script_path.is_file():
                raise FileNotFoundError(f"controller script not found: {self.script_path}")
            actual_hash = _sha256_file(self.script_path)
            if actual_hash != CONTROLLER_SHA256:
                raise RuntimeError(
                    "controller script hash mismatch; refusing launch. "
                    f"expected {CONTROLLER_SHA256}, got {actual_hash or 'unreadable'}"
                )
            if not (self.python_path.is_file() and os.access(self.python_path, os.X_OK)):
                raise FileNotFoundError(f"controller Python not executable: {self.python_path}")
            matching = _find_matching_pids(self.script_path)
            if matching:
                raise RuntimeError(f"controller already running (pid {matching[0]})")

            cmd, params = self.build_command(parameters)
            env = _controller_environment(self.python_path)
            controller_action_token = secrets.token_urlsafe(24)
            env["G1_DASHBOARD_MANAGED"] = "1"
            env["G1_DASHBOARD_ACTION_TOKEN"] = controller_action_token
            preflight = _controller_python_preflight(
                self.python_path, env, self.script_path.parent
            )

            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            log = open(self.log_path, "ab", buffering=0)
            banner = (
                f"\n===== dashboard launch {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n"
                f"command: {' '.join(cmd)}\n"
                "managed_env: PYTHONNOUSERSITE=1; PYTHONPATH/PYTHONHOME cleared; keyboard disabled; loopback XR actions enabled\n"
                f"preflight:\n{preflight}\n"
            ).encode("utf-8", "replace")
            log.write(banner)
            try:
                proc = subprocess.Popen(
                    cmd,
                    cwd=str(self.script_path.parent),
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env=env,
                    start_new_session=True,
                    close_fds=True,
                )
            except Exception:
                log.close()
                raise
            log.close()
            self._popen = proc
            ticks = None
            # /proc can appear a few milliseconds after Popen returns.
            for _ in range(20):
                ticks = _proc_start_ticks(proc.pid)
                if ticks is not None:
                    break
                time.sleep(0.005)
            self._managed = {
                "pid": proc.pid,
                "start_ticks": ticks,
                "launched_unix_time_s": time.time(),
                "parameters": params,
                "command": cmd,
                "stop_requested_unix_time_s": None,
                "action_token": controller_action_token,
            }
            self._last_error = None
            self._last_exit_code = None
            self._save_state()
            # Detect immediate import/startup failures without blocking normal launches.
            time.sleep(0.08)
            rc = proc.poll()
            if rc is not None:
                self._last_exit_code = int(rc)
                self._managed = None
                self._popen = None
                self._save_state()
                raise RuntimeError(f"controller exited immediately with code {rc}; inspect {self.log_path}")
            return self.status()

    def request_xr_action(self, operation: str) -> dict[str, Any]:
        """Request one controller-owned transition; never touches DDS itself."""
        operation = str(operation).strip()
        if operation not in ACTION_OPERATIONS:
            raise ValueError(f"unsupported XR action {operation!r}")

        with self._lock:
            if not self.enabled:
                raise PermissionError("controller process actions are disabled")
            self._refresh()
            alive, pid = self._managed_alive()
            if not alive or pid is None or self._managed is None:
                raise RuntimeError("dashboard-managed controller is not running")
            if self._managed.get("stop_requested_unix_time_s"):
                raise RuntimeError("controller stop/handback is already pending")
            token = str(self._managed.get("action_token", ""))
            if len(token) < 16:
                raise RuntimeError(
                    "running controller predates the Step 5.1 action token; restart it from this dashboard"
                )

        request_id = uuid.uuid4().hex
        payload = {
            "schema": ACTION_REQUEST_SCHEMA,
            "request_id": request_id,
            "operation": operation,
            "token": token,
        }
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((ACTION_HOST, 0))
            sock.settimeout(0.8)
            sock.sendto(encoded, (ACTION_HOST, ACTION_PORT))
            raw, address = sock.recvfrom(4096)
        except socket.timeout as exc:
            raise RuntimeError(
                "controller action channel did not respond; confirm the managed controller reached startup"
            ) from exc
        finally:
            sock.close()

        if address[0] != ACTION_HOST:
            raise RuntimeError("controller action response was not loopback")
        try:
            response = json.loads(raw.decode("utf-8"))
        except Exception as exc:
            raise RuntimeError(f"invalid controller action response: {exc}") from exc
        if not isinstance(response, dict):
            raise RuntimeError("invalid controller action response object")
        if response.get("schema") != ACTION_RESPONSE_SCHEMA:
            raise RuntimeError("controller action response schema mismatch")
        if response.get("request_id") != request_id:
            raise RuntimeError("controller action response request_id mismatch")
        return response

    def stop(self) -> dict[str, Any]:
        with self._lock:
            if not self.enabled:
                raise PermissionError("controller process actions are disabled")
            self._refresh()
            alive, pid = self._managed_alive()
            if not alive or pid is None or self._managed is None:
                matching = _find_matching_pids(self.script_path)
                if matching:
                    raise PermissionError(
                        f"controller pid {matching[0]} was not launched/adopted by this dashboard; stop it from its owning terminal"
                    )
                return self.status()
            if self._managed.get("stop_requested_unix_time_s"):
                return self.status()
            # One SIGTERM requests the controller's normal controlled handback.
            # It must not be replaced with SIGKILL: the controller owns release.
            os.kill(pid, signal.SIGTERM)
            self._managed["stop_requested_unix_time_s"] = time.time()
            self._save_state()
            return self.status()

    def log_tail(self, max_lines: int = 80) -> dict[str, Any]:
        max_lines = max(1, min(200, int(max_lines)))
        try:
            if not self.log_path.exists():
                lines: list[str] = []
            else:
                # Log sizes are small in normal use; cap the read to the final 128 KiB.
                with self.log_path.open("rb") as f:
                    try:
                        f.seek(0, os.SEEK_END)
                        size = f.tell()
                        f.seek(max(0, size - 131072), os.SEEK_SET)
                    except OSError:
                        pass
                    text = f.read().decode("utf-8", "replace")
                lines = text.splitlines()[-max_lines:]
        except Exception as exc:
            lines = [f"log unavailable: {exc}"]
        return {"schema": MANAGER_SCHEMA, "log_path": str(self.log_path), "lines": lines}
