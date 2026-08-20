#!/usr/bin/env python3
"""Whitelisted lifecycle manager for G1 XR controller dependencies.

This module intentionally does not import Unitree DDS libraries. It can only
start/stop the exact validated controller, the fixed dashboard Teleimager/RealSense
runner in the validated teleimager environment, and the installed root-owned Inspire helper. The browser never supplies an
executable path, shell fragment, environment variable, PID, or arbitrary
command-line token.
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
MANAGER_VERSION = "g1_dashboard_process_manager.v1.3.0-realsense-camera-modes"
CONTROLLER_SHA256 = "1e5d92c3c460c4f652e22e8de00ae2305d444f3e4a2485dfa8cebbb68c2b8484"

ACTION_REQUEST_SCHEMA = "g1_dashboard.action_request.v1"
ACTION_RESPONSE_SCHEMA = "g1_dashboard.action_response.v1"
ACTION_HOST = "127.0.0.1"
ACTION_PORT = 8767
ACTION_OPERATIONS = {"REQUEST_XR", "CANCEL_XR_REQUEST", "HAND_BACK_ARMS"}

INSPIRE_SERVICE_BASENAME = "inspire_g1"
CAMERA_BASENAME = "teleimager-server"
CAMERA_RUNNER_BASENAME = "g1_dashboard_teleimager_modes_runner.py"
CAMERA_WEBRTC_PORT = 60001
CAMERA_DISPLAY_MODES = {"rgb", "depth", "overlay", "near"}

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


def _proc_pgid(pid: int) -> int | None:
    """Return a process-group id without treating worker children as conflicts."""
    try:
        return int(os.getpgid(int(pid)))
    except (ProcessLookupError, PermissionError, OSError, ValueError):
        return None


def _group_pids_by_pgid(pids: list[int]) -> dict[int, list[int]]:
    groups: dict[int, list[int]] = {}
    for pid in sorted(set(int(p) for p in pids)):
        pgid = _proc_pgid(pid)
        # A process that vanished between /proc scans should not create a fake
        # conflict group. The next status refresh will rescan it.
        if pgid is None:
            continue
        groups.setdefault(pgid, []).append(pid)
    return groups


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


def _find_pids_by_basename(basename: str) -> list[int]:
    out: list[int] = []
    proc = Path("/proc")
    if not proc.exists():
        return out
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        args = _proc_cmdline(pid)
        if any(Path(arg).name == basename for arg in args):
            out.append(pid)
    return sorted(set(out))


def _pids_in_process_group(pgid: int | None) -> list[int]:
    if pgid is None:
        return []
    out: list[int] = []
    proc = Path("/proc")
    if not proc.is_dir():
        return out
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if _proc_pgid(pid) == pgid:
            out.append(pid)
    return sorted(set(out))


def _find_camera_pids() -> list[int]:
    return sorted(set(_find_pids_by_basename(CAMERA_BASENAME)) | set(_find_pids_by_basename(CAMERA_RUNNER_BASENAME)))


def _tcp_port_open(host: str, port: int, timeout_s: float = 0.08) -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.settimeout(timeout_s)
        return sock.connect_ex((host, int(port))) == 0
    except Exception:
        return False
    finally:
        sock.close()


def default_camera_executable() -> Path:
    override = os.environ.get("G1_DASHBOARD_CAMERA_EXECUTABLE", "").strip()
    if override:
        path = Path(override).expanduser()
        if path.name != CAMERA_BASENAME:
            raise RuntimeError(f"camera executable must be {CAMERA_BASENAME}; got {path.name!r}")
        return path
    candidates = [
        Path.home() / "miniconda3" / "envs" / "teleimager" / "bin" / CAMERA_BASENAME,
        Path.home() / "miniforge3" / "envs" / "teleimager" / "bin" / CAMERA_BASENAME,
        Path.home() / "anaconda3" / "envs" / "teleimager" / "bin" / CAMERA_BASENAME,
    ]
    for path in candidates:
        if path.is_file() and os.access(path, os.X_OK):
            return path
    return candidates[0]


def _camera_environment(executable: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONNOUSERSITE"] = "1"
    # Do NOT put all of /usr/lib/python3/dist-packages on PYTHONPATH.  Doing
    # that can shadow Conda's cryptography stack with Ubuntu packages while
    # aiortc still imports Conda's pyOpenSSL, producing an incompatible mix.
    # The camera runner temporarily exposes this directory only long enough to
    # preload the already-installed pyrealsense2 package, then removes it from
    # sys.path before Teleimager / aiortc are imported.
    env.pop("PYTHONPATH", None)
    env["G1_DASHBOARD_REALSENSE_SYSTEM_SITE"] = os.environ.get(
        "G1_DASHBOARD_REALSENSE_SYSTEM_SITE",
        "/usr/lib/python3/dist-packages",
    )
    env.pop("PYTHONHOME", None)
    prefix = executable.parent.parent
    env["PATH"] = f"{prefix / 'bin'}:{env.get('PATH', '')}"
    env["CONDA_PREFIX"] = str(prefix)
    env["CONDA_DEFAULT_ENV"] = prefix.name
    ld = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = "/usr/local/lib" + (f":{ld}" if ld else "")
    return env


def _camera_preflight(executable: Path, cwd: Path, env: dict[str, str]) -> str:
    """Import the server in a fresh isolated interpreter before spawning it."""
    python = executable.parent / "python"
    if not (python.is_file() and os.access(python, os.X_OK)):
        raise RuntimeError(f"teleimager Python is not executable: {python}")
    try:
        result = subprocess.run(
            [
                str(python),
                "-c",
                (
                    "import os,sys; "
                    "p=os.environ.get('G1_DASHBOARD_REALSENSE_SYSTEM_SITE','/usr/lib/python3/dist-packages'); "
                    "sys.path.insert(0,p); "
                    "import pyrealsense2 as rs; "
                    "sys.path.remove(p); "
                    "import teleimager.image_server; "
                    "ctx=rs.context(); "
                    "print('teleimager.image_server=OK'); "
                    "print('pyrealsense2=OK devices=%d' % len(ctx.query_devices()))"
                ),
            ],
            cwd=str(cwd),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=10.0,
            check=False,
            text=True,
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("teleimager dependency preflight timed out") from exc
    output = (result.stdout or "").strip()
    if result.returncode != 0:
        tail = "\n".join(output.splitlines()[-12:]) if output else f"python exit {result.returncode}"
        raise RuntimeError(f"teleimager dependency preflight failed:\n{tail}")
    return output or "teleimager.image_server=OK"


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

        self.inspire_helper_path = Path(os.environ.get(
            "G1_DASHBOARD_INSPIRE_HELPER",
            "/usr/local/libexec/g1-dashboard/g1_dashboard_inspire_helper.py",
        ))
        self.camera_executable = default_camera_executable()
        self.camera_cwd = Path(os.environ.get(
            "G1_DASHBOARD_CAMERA_CWD",
            str(Path.home() / "teleimager"),
        )).expanduser()
        self.camera_runner = Path(os.environ.get(
            "G1_DASHBOARD_CAMERA_RUNNER",
            str(Path(__file__).with_name(CAMERA_RUNNER_BASENAME)),
        )).expanduser()
        self.camera_config = Path(os.environ.get(
            "G1_DASHBOARD_CAMERA_CONFIG",
            str(Path(__file__).with_name("cam_config_realsense_modes.yaml")),
        )).expanduser()
        self.camera_mode_path = Path(os.environ.get(
            "G1_DASHBOARD_CAMERA_MODE_FILE",
            f"/tmp/g1_dashboard_camera_mode_{os.getuid()}.txt",
        )).expanduser()
        self.camera_mode_status_path = Path(os.environ.get(
            "G1_DASHBOARD_CAMERA_MODE_STATUS",
            f"/tmp/g1_dashboard_camera_mode_status_{os.getuid()}.json",
        )).expanduser()
        self.camera_state_path = Path(os.environ.get(
            "G1_DASHBOARD_CAMERA_STATE",
            "/tmp/g1_dashboard_camera_teleimager_state.json",
        ))
        self.camera_log_path = Path(os.environ.get(
            "G1_DASHBOARD_CAMERA_LOG",
            str(Path.home() / ".local" / "state" / "g1_dashboard" / "camera_teleimager.log"),
        )).expanduser()

        self._lock = threading.RLock()
        self._popen: subprocess.Popen[bytes] | None = None
        self._managed: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._last_exit_code: int | None = None
        self._camera_popen: subprocess.Popen[bytes] | None = None
        self._camera_managed: dict[str, Any] | None = None
        self._camera_last_error: str | None = None
        self._camera_last_exit_code: int | None = None
        self._load_state()
        self._load_camera_state()

    # ---------- persisted controller state ----------
    def _load_state(self) -> None:
        stale: dict[str, Any] | None = None
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
            stale = data
        except Exception:
            pass
        self._managed = None
        try:
            self.state_path.unlink(missing_ok=True)
        except Exception:
            pass
        # If the dashboard restarted during an explicit controller stop, finish
        # the dependency cleanup only after confirming the controller is gone.
        if stale and stale.get("stop_requested_unix_time_s") and stale.get("inspire_stop_with_controller"):
            try:
                self._stop_inspire_managed_dependency()
            except Exception as exc:
                self._last_error = f"controller stopped, but Inspire cleanup failed after dashboard restart: {exc}"

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

    # ---------- Inspire privileged dependency ----------
    def _inspire_status(self) -> dict[str, Any]:
        helper = self.inspire_helper_path
        if helper.is_file() and os.access(helper, os.X_OK):
            try:
                result = subprocess.run(
                    [str(helper), "status"],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=1.5,
                    check=False,
                    text=True,
                )
                text = (result.stdout or "").strip()
                data = json.loads(text.splitlines()[-1]) if text else {}
                if result.returncode == 0 and isinstance(data, dict):
                    data["helper_installed"] = True
                    data["helper_path"] = str(helper)
                    return data
                return {
                    "schema": "g1_dashboard.inspire_service.v1",
                    "state": "UNAVAILABLE",
                    "managed": False,
                    "pid": None,
                    "external_pids": _find_pids_by_basename(INSPIRE_SERVICE_BASENAME),
                    "helper_installed": True,
                    "helper_path": str(helper),
                    "error": data.get("error") if isinstance(data, dict) else text or f"helper exit {result.returncode}",
                }
            except Exception as exc:
                return {
                    "schema": "g1_dashboard.inspire_service.v1",
                    "state": "UNAVAILABLE",
                    "managed": False,
                    "pid": None,
                    "external_pids": _find_pids_by_basename(INSPIRE_SERVICE_BASENAME),
                    "helper_installed": True,
                    "helper_path": str(helper),
                    "error": str(exc),
                }

        external = _find_pids_by_basename(INSPIRE_SERVICE_BASENAME)
        if len(external) == 1:
            state = "RUNNING_EXTERNAL"
        elif len(external) > 1:
            state = "CONFLICT"
        else:
            state = "UNAVAILABLE"
        return {
            "schema": "g1_dashboard.inspire_service.v1",
            "state": state,
            "managed": False,
            "pid": external[0] if len(external) == 1 else None,
            "external_pids": external,
            "helper_installed": False,
            "helper_path": str(helper),
            "error": None if external else "privileged Inspire helper is not installed",
        }

    def _run_inspire_privileged(self, action: str, timeout: float = 8.0) -> dict[str, Any]:
        if action not in {"start", "stop"}:
            raise ValueError("unsupported Inspire helper action")
        helper = self.inspire_helper_path
        if not (helper.is_file() and os.access(helper, os.X_OK)):
            raise RuntimeError(
                "Inspire lifecycle helper is not installed. Run sudo ./install_inspire_helper.sh once on PC2."
            )
        sudo = Path("/usr/bin/sudo")
        if not sudo.is_file():
            raise RuntimeError("/usr/bin/sudo not found; cannot use the installed Inspire lifecycle helper")
        try:
            result = subprocess.run(
                [str(sudo), "-n", str(helper), action],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                check=False,
                text=True,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"Inspire {action} helper timed out") from exc
        text = (result.stdout or "").strip()
        data: dict[str, Any] = {}
        if text:
            try:
                candidate = json.loads(text.splitlines()[-1])
                if isinstance(candidate, dict):
                    data = candidate
            except Exception:
                pass
        if result.returncode != 0:
            detail = data.get("error") or text or f"sudo/helper exit {result.returncode}"
            if "password" in detail.lower() or "sudoers" in detail.lower() or "not allowed" in detail.lower():
                detail += "; run sudo ./install_inspire_helper.sh once on PC2"
            raise RuntimeError(f"Inspire {action} failed: {detail}")
        return data if data else self._inspire_status()

    def _ensure_inspire_for_controller(self) -> tuple[dict[str, Any], bool]:
        """Return (status, newly_started_by_this_call)."""
        st = self._inspire_status()
        state = st.get("state")
        if state == "RUNNING_MANAGED":
            return st, False
        if state == "RUNNING_EXTERNAL":
            return st, False
        if state == "CONFLICT":
            raise RuntimeError(f"multiple/conflicting Inspire processes detected: {st.get('external_pids', [])}")
        if not st.get("helper_installed"):
            raise RuntimeError(
                "Inspire server is not running and the privileged lifecycle helper is not installed. "
                "Run sudo ./install_inspire_helper.sh once on PC2."
            )
        started = self._run_inspire_privileged("start")
        if started.get("state") != "RUNNING_MANAGED":
            raise RuntimeError(f"Inspire helper did not reach RUNNING_MANAGED: {started}")
        return started, True

    def _stop_inspire_managed_dependency(self) -> dict[str, Any]:
        st = self._inspire_status()
        if st.get("state") == "RUNNING_MANAGED":
            return self._run_inspire_privileged("stop")
        # Never stop a service that was not started by the root-owned helper.
        return st

    # ---------- camera process state ----------
    def _load_camera_state(self) -> None:
        try:
            data = json.loads(self.camera_state_path.read_text())
            pid = int(data.get("pid", 0))
            ticks = data.get("start_ticks")
            ticks = int(ticks) if ticks is not None else None
            if _process_alive(pid, ticks) and any(
                Path(a).name in {CAMERA_BASENAME, CAMERA_RUNNER_BASENAME}
                for a in _proc_cmdline(pid)
            ):
                self._camera_managed = data
                return
        except Exception:
            pass
        self._camera_managed = None
        try:
            self.camera_state_path.unlink(missing_ok=True)
        except Exception:
            pass

    def _save_camera_state(self) -> None:
        if self._camera_managed is None:
            try:
                self.camera_state_path.unlink(missing_ok=True)
            except Exception:
                pass
            return
        try:
            self.camera_state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.camera_state_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(self._camera_managed, indent=2, sort_keys=True))
            os.chmod(tmp, 0o600)
            tmp.replace(self.camera_state_path)
        except Exception as exc:
            self._camera_last_error = f"failed to persist camera process state: {exc}"

    def _camera_managed_alive(self) -> tuple[bool, int | None]:
        if self._camera_managed is None:
            return False, None
        pid = int(self._camera_managed.get("pid", 0))
        ticks = self._camera_managed.get("start_ticks")
        ticks = int(ticks) if ticks is not None else None
        return _process_alive(pid, ticks), pid

    def _refresh_camera(self) -> None:
        if self._camera_popen is not None:
            rc = self._camera_popen.poll()
            if rc is not None:
                self._camera_last_exit_code = int(rc)
                self._camera_popen = None
        alive, pid = self._camera_managed_alive()
        if self._camera_managed is not None and alive and pid is not None:
            requested = self._camera_managed.get("stop_requested_unix_time_s")
            if requested and not self._camera_managed.get("term_sent_unix_time_s"):
                if time.time() - float(requested) >= 3.0:
                    try:
                        pgid = _proc_pgid(pid)
                        os.killpg(pgid if pgid is not None else pid, signal.SIGTERM)
                        self._camera_managed["term_sent_unix_time_s"] = time.time()
                        self._save_camera_state()
                    except ProcessLookupError:
                        pass
                    except Exception as exc:
                        self._camera_last_error = f"camera SIGTERM fallback failed: {exc}"
        alive, _pid = self._camera_managed_alive()
        if self._camera_managed is not None and not alive:
            self._camera_managed = None
            self._save_camera_state()

    def _read_camera_mode_requested(self) -> str:
        try:
            mode = self.camera_mode_path.read_text(encoding="utf-8").strip().lower()
            return mode if mode in CAMERA_DISPLAY_MODES else "rgb"
        except Exception:
            return "rgb"

    def _read_camera_mode_status(self) -> dict[str, Any] | None:
        try:
            data = json.loads(self.camera_mode_status_path.read_text(encoding="utf-8"))
            if data.get("schema") != "g1_dashboard.camera_display_status.v1":
                return None
            updated = float(data.get("updated_unix_time_s", 0.0))
            data["online"] = bool(updated > 0 and time.time() - updated <= 2.0)
            return data
        except Exception:
            return None

    def _write_camera_mode(self, mode: str) -> str:
        normalized = str(mode or "").strip().lower()
        if normalized not in CAMERA_DISPLAY_MODES:
            raise ValueError(f"camera mode must be one of {sorted(CAMERA_DISPLAY_MODES)}")
        self.camera_mode_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.camera_mode_path.with_suffix(".tmp")
        tmp.write_text(normalized + "\n", encoding="utf-8")
        os.chmod(tmp, 0o600)
        tmp.replace(self.camera_mode_path)
        return normalized

    def set_camera_mode(self, mode: str) -> dict[str, Any]:
        with self._lock:
            if not self.enabled:
                raise PermissionError("dashboard process actions are disabled")
            status = self.camera_status()
            if status.get("state") != "RUNNING" or not status.get("mode_control"):
                raise PermissionError(
                    "camera display modes require the dashboard-managed RealSense mode runner; "
                    "stop/restart an older managed teleimager process first"
                )
            self._write_camera_mode(mode)
            # Do not restart WebRTC. The runner polls this local file and changes
            # only the BGR frame written into Teleimager's existing buffer.
            deadline = time.monotonic() + 1.2
            requested = self._read_camera_mode_requested()
            while time.monotonic() < deadline:
                ack = self._read_camera_mode_status()
                if ack and ack.get("online") and ack.get("mode") == requested:
                    break
                time.sleep(0.03)
            return self.camera_status()

    def camera_status(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_camera()
            matching = _find_camera_pids()
            groups = _group_pids_by_pgid(matching)
            managed_alive, managed_pid = self._camera_managed_alive()
            managed_pgid = _proc_pgid(managed_pid) if managed_alive and managed_pid is not None else None
            now = time.time()
            executable_ok = self.camera_executable.is_file() and os.access(self.camera_executable, os.X_OK)
            cwd_ok = self.camera_cwd.is_dir()
            port_ready = _tcp_port_open("127.0.0.1", CAMERA_WEBRTC_PORT)
            mode_requested = self._read_camera_mode_requested()
            mode_status = self._read_camera_mode_status()
            mode_actual = mode_status.get("mode") if mode_status and mode_status.get("online") else None
            mode_ack_online = bool(mode_status and mode_status.get("online"))

            if managed_alive and self._camera_managed is not None:
                # teleimager uses multiprocessing and normally creates one or
                # more worker processes with the same `teleimager-server`
                # basename. Because the dashboard launches the server in a new
                # session, every normal worker belongs to the managed process
                # group. Only another process group is an external conflict.
                managed_family = _pids_in_process_group(managed_pgid) if managed_pgid is not None else [managed_pid]
                managed_mode_runner = any(
                    Path(a).name == CAMERA_RUNNER_BASENAME for a in _proc_cmdline(managed_pid)
                )
                external_groups = {pgid: pids for pgid, pids in groups.items() if pgid != managed_pgid}
                external = sorted(pid for pids in external_groups.values() for pid in pids)
                stop_requested = self._camera_managed.get("stop_requested_unix_time_s")
                state = "STOPPING" if stop_requested else ("CONFLICT" if external_groups else "RUNNING")
                launched = float(self._camera_managed.get("launched_unix_time_s", now))
                return {
                    "schema": "g1_dashboard.camera_process.v1",
                    "manager_version": MANAGER_VERSION,
                    "enabled": self.enabled,
                    "state": state,
                    "managed": True,
                    "pid": managed_pid,
                    "pgid": managed_pgid,
                    "process_pids": managed_family,
                    "uptime_s": max(0.0, now - launched),
                    "stop_requested_unix_time_s": stop_requested,
                    "external_pids": external,
                    "external_process_groups": {str(k): v for k, v in external_groups.items()},
                    "executable": str(self.camera_executable),
                    "cwd": str(self.camera_cwd),
                    "log_path": str(self.camera_log_path),
                    "port": CAMERA_WEBRTC_PORT,
                    "port_ready": port_ready,
                    "display_modes": sorted(CAMERA_DISPLAY_MODES),
                    "mode_requested": mode_requested,
                    "mode_actual": mode_actual,
                    "mode_ack_online": mode_ack_online,
                    "mode_control": state == "RUNNING" and not external_groups and managed_mode_runner,
                    "mode_status": mode_status,
                    "last_error": self._camera_last_error,
                    "last_exit_code": self._camera_last_exit_code,
                    "can_start": False,
                    "can_stop": state == "RUNNING",
                }

            if groups:
                external = sorted(pid for pids in groups.values() for pid in pids)
                state = "RUNNING_EXTERNAL" if len(groups) == 1 else "CONFLICT"
                only_pgid = next(iter(groups)) if len(groups) == 1 else None
                return {
                    "schema": "g1_dashboard.camera_process.v1",
                    "manager_version": MANAGER_VERSION,
                    "enabled": self.enabled,
                    "state": state,
                    "managed": False,
                    "pid": external[0] if len(groups) == 1 and external else None,
                    "pgid": only_pgid,
                    "process_pids": external if len(groups) == 1 else [],
                    "uptime_s": None,
                    "external_pids": external,
                    "external_process_groups": {str(k): v for k, v in groups.items()},
                    "executable": str(self.camera_executable),
                    "cwd": str(self.camera_cwd),
                    "log_path": str(self.camera_log_path),
                    "port": CAMERA_WEBRTC_PORT,
                    "port_ready": port_ready,
                    "display_modes": sorted(CAMERA_DISPLAY_MODES),
                    "mode_requested": mode_requested,
                    "mode_actual": mode_actual,
                    "mode_ack_online": mode_ack_online,
                    "mode_control": False,
                    "mode_status": mode_status,
                    "last_error": self._camera_last_error,
                    "last_exit_code": self._camera_last_exit_code,
                    "can_start": False,
                    "can_stop": False,
                }

            ready = self.enabled and executable_ok and cwd_ok and self.camera_runner.is_file() and self.camera_config.is_file()
            return {
                "schema": "g1_dashboard.camera_process.v1",
                "manager_version": MANAGER_VERSION,
                "enabled": self.enabled,
                "state": "STOPPED" if ready else "UNAVAILABLE",
                "managed": False,
                "pid": None,
                "pgid": None,
                "process_pids": [],
                "uptime_s": None,
                "external_pids": [],
                "external_process_groups": {},
                "executable": str(self.camera_executable),
                "cwd": str(self.camera_cwd),
                "log_path": str(self.camera_log_path),
                "port": CAMERA_WEBRTC_PORT,
                "port_ready": port_ready,
                "display_modes": sorted(CAMERA_DISPLAY_MODES),
                "mode_requested": mode_requested,
                "mode_actual": mode_actual,
                "mode_ack_online": mode_ack_online,
                "mode_control": False,
                "mode_status": mode_status,
                "last_error": self._camera_last_error,
                "last_exit_code": self._camera_last_exit_code,
                "can_start": ready,
                "can_stop": False,
            }

    def start_camera(self) -> dict[str, Any]:
        with self._lock:
            if not self.enabled:
                raise PermissionError("dashboard process actions are disabled")
            st = self.camera_status()
            if st["state"] in {"RUNNING", "RUNNING_EXTERNAL"}:
                return st
            if st["state"] == "CONFLICT":
                raise RuntimeError(f"multiple/conflicting teleimager processes detected: {st.get('external_pids', [])}")
            if not (self.camera_executable.is_file() and os.access(self.camera_executable, os.X_OK)):
                raise FileNotFoundError(f"teleimager-server not executable: {self.camera_executable}")
            if not self.camera_cwd.is_dir():
                raise FileNotFoundError(f"teleimager working directory not found: {self.camera_cwd}")
            if not self.camera_runner.is_file():
                raise FileNotFoundError(f"dashboard camera runner not found: {self.camera_runner}")
            if not self.camera_config.is_file():
                raise FileNotFoundError(f"dashboard RealSense camera config not found: {self.camera_config}")
            matching = _find_camera_pids()
            if matching:
                return self.camera_status()

            env = _camera_environment(self.camera_executable)
            env["G1_DASHBOARD_CAMERA_CONFIG"] = str(self.camera_config)
            env["TELEIMAGER_DISPLAY_MODE_FILE"] = str(self.camera_mode_path)
            env["TELEIMAGER_DISPLAY_STATUS_FILE"] = str(self.camera_mode_status_path)
            # Every camera-server launch returns to the normal RGB operator
            # view. Mode changes are live-session choices, not boot defaults.
            self._write_camera_mode("rgb")
            try:
                self.camera_mode_status_path.unlink(missing_ok=True)
            except Exception:
                pass
            try:
                preflight = _camera_preflight(self.camera_executable, self.camera_cwd, env)
            except Exception as exc:
                self._camera_last_error = str(exc)
                raise
            self.camera_log_path.parent.mkdir(parents=True, exist_ok=True)
            log = open(self.camera_log_path, "ab", buffering=0)
            log.write(
                (
                    f"\n===== dashboard camera launch {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n"
                    f"command: {self.camera_executable.parent / 'python'} {self.camera_runner} --rs\n"
                    "managed_env: teleimager conda prefix; PYTHONNOUSERSITE=1; pyrealsense2 preloaded selectively (no system-site PYTHONPATH)\n"
                    f"preflight: {preflight}\n"
                ).encode("utf-8", "replace")
            )
            try:
                camera_python = self.camera_executable.parent / "python"
                proc = subprocess.Popen(
                    [str(camera_python), str(self.camera_runner), "--rs"],
                    cwd=str(self.camera_cwd),
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
            self._camera_popen = proc
            ticks = None
            for _ in range(20):
                ticks = _proc_start_ticks(proc.pid)
                if ticks is not None:
                    break
                time.sleep(0.005)
            self._camera_managed = {
                "pid": proc.pid,
                "pgid": _proc_pgid(proc.pid),
                "start_ticks": ticks,
                "launched_unix_time_s": time.time(),
                "stop_requested_unix_time_s": None,
                "term_sent_unix_time_s": None,
            }
            self._camera_last_error = None
            self._camera_last_exit_code = None
            self._save_camera_state()
            time.sleep(0.12)
            rc = proc.poll()
            if rc is not None:
                self._camera_last_exit_code = int(rc)
                self._camera_managed = None
                self._camera_popen = None
                self._save_camera_state()
                raise RuntimeError(
                    f"teleimager-server exited immediately with code {rc}; inspect {self.camera_log_path}"
                )
            return self.camera_status()

    def stop_camera(self) -> dict[str, Any]:
        with self._lock:
            if not self.enabled:
                raise PermissionError("dashboard process actions are disabled")
            self._refresh_camera()
            alive, pid = self._camera_managed_alive()
            if not alive or pid is None or self._camera_managed is None:
                matching = _find_camera_pids()
                if matching:
                    raise PermissionError(
                        f"teleimager pid {matching[0]} was not launched/adopted by this dashboard; disconnect only or stop it from its owning terminal"
                    )
                return self.camera_status()
            if self._camera_managed.get("stop_requested_unix_time_s"):
                return self.camera_status()
            # Validated manual shutdown is Ctrl+C, so use SIGINT for the
            # entire managed teleimager process group (parent + workers).
            pgid = _proc_pgid(pid)
            try:
                os.killpg(pgid if pgid is not None else pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            self._camera_managed["stop_requested_unix_time_s"] = time.time()
            self._save_camera_state()
            return self.camera_status()

    # ---------- controller configuration ----------
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
        inspire = self._inspire_status()
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
            "locked_settings": list(LOCKED_SETTINGS) + [
                {"label": "Inspire service", "value": "starts before listener · stops after controlled listener shutdown"},
            ],
            "parameter_specs": PARAMETER_SPECS,
            "known_good": {spec["name"]: spec["default"] for spec in PARAMETER_SPECS},
            "dashboard_edit_bounds_are_safety_limits": False,
            "management_key_required": True,
            "inspire_dependency": inspire,
            "xr_action_channel": {
                "transport": f"udp://{ACTION_HOST}:{ACTION_PORT}",
                "loopback_only": True,
                "controller_validated": True,
                "operations": sorted(ACTION_OPERATIONS),
            },
            "camera_process": {
                "executable": str(self.camera_executable),
                "cwd": str(self.camera_cwd),
                "webrtc_port": CAMERA_WEBRTC_PORT,
                "managed_from_camera_buttons": True,
            },
            "log_path": str(self.log_path),
        }

    # ---------- controller lifecycle ----------
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
            finished = self._managed
            self._managed = None
            self._save_state()
            if finished.get("stop_requested_unix_time_s") and finished.get("inspire_stop_with_controller"):
                try:
                    self._stop_inspire_managed_dependency()
                except Exception as exc:
                    self._last_error = f"controller stopped, but Inspire dependency stop failed: {exc}"
            elif finished.get("inspire_stop_with_controller"):
                # Unexpected controller exit: do not silently kill a hardware service.
                self._last_error = (
                    "controller exited without an explicit dashboard stop; dashboard-managed Inspire service was left running for operator inspection"
                )

    def status(self) -> dict[str, Any]:
        with self._lock:
            self._refresh()
            matching = _find_matching_pids(self.script_path) if self.script_path.exists() else []
            managed_alive, managed_pid = self._managed_alive()
            external = [pid for pid in matching if pid != managed_pid]
            now = time.time()
            inspire = self._inspire_status()

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
                    "dependencies": {"inspire": inspire},
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
                    "dependencies": {"inspire": inspire},
                }

            script_hash = _sha256_file(self.script_path) if self.script_path.is_file() else None
            inspire_state = inspire.get("state")
            inspire_startable = inspire_state in {"RUNNING_MANAGED", "RUNNING_EXTERNAL"} or (
                inspire_state in {"STOPPED", "UNAVAILABLE"}
                and bool(inspire.get("helper_installed"))
                and inspire.get("binary_exists", True) is not False
            )
            if inspire_state == "CONFLICT":
                inspire_startable = False
            ready = (
                self.enabled
                and self.script_path.is_file()
                and script_hash == CONTROLLER_SHA256
                and self.python_path.is_file()
                and os.access(self.python_path, os.X_OK)
                and inspire_startable
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
                "dependencies": {"inspire": inspire},
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

            # Hardware dependency is started only after Python/controller preflight
            # succeeds, but always before the teleop controller itself.
            inspire, inspire_started_now = self._ensure_inspire_for_controller()
            inspire_stop_with_controller = inspire.get("state") == "RUNNING_MANAGED"

            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            log = open(self.log_path, "ab", buffering=0)
            banner = (
                f"\n===== dashboard launch {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n"
                f"command: {' '.join(cmd)}\n"
                "managed_env: PYTHONNOUSERSITE=1; PYTHONPATH/PYTHONHOME cleared; keyboard disabled; loopback XR actions enabled\n"
                f"inspire_dependency: state={inspire.get('state')} pid={inspire.get('pid')} stop_with_controller={inspire_stop_with_controller}\n"
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
                if inspire_started_now:
                    try:
                        self._stop_inspire_managed_dependency()
                    except Exception as cleanup_exc:
                        self._last_error = f"controller spawn failed and Inspire rollback also failed: {cleanup_exc}"
                raise
            log.close()
            self._popen = proc
            ticks = None
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
                "inspire_stop_with_controller": inspire_stop_with_controller,
            }
            self._last_error = None
            self._last_exit_code = None
            self._save_state()
            time.sleep(0.08)
            rc = proc.poll()
            if rc is not None:
                self._last_exit_code = int(rc)
                self._managed = None
                self._popen = None
                self._save_state()
                if inspire_started_now:
                    try:
                        self._stop_inspire_managed_dependency()
                    except Exception as cleanup_exc:
                        self._last_error = f"controller exited immediately and Inspire rollback failed: {cleanup_exc}"
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

    def _watch_controller_stop_cleanup(self, pid: int, ticks: int | None) -> None:
        """Finish Inspire cleanup after an explicit listener stop without relying on browser polling."""
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and _process_alive(pid, ticks):
            time.sleep(0.10)
        if _process_alive(pid, ticks):
            with self._lock:
                self._last_error = (
                    f"controller pid {pid} did not exit within 30 s after stop request; Inspire was left running"
                )
            return
        with self._lock:
            current = self._managed
            if not current or int(current.get("pid", 0)) != pid:
                return
            if not current.get("stop_requested_unix_time_s"):
                return
            should_stop_inspire = bool(current.get("inspire_stop_with_controller"))
            if self._popen is not None:
                rc = self._popen.poll()
                if rc is not None:
                    self._last_exit_code = int(rc)
                    self._popen = None
            self._managed = None
            self._save_state()
            if should_stop_inspire:
                try:
                    self._stop_inspire_managed_dependency()
                except Exception as exc:
                    self._last_error = f"controller stopped, but Inspire dependency stop failed: {exc}"

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
            # Controller must stop first.  _refresh() stops a helper-managed
            # Inspire service only after the controlled controller process exits.
            ticks = self._managed.get("start_ticks")
            ticks = int(ticks) if ticks is not None else None
            os.kill(pid, signal.SIGTERM)
            self._managed["stop_requested_unix_time_s"] = time.time()
            self._save_state()
            threading.Thread(
                target=self._watch_controller_stop_cleanup,
                args=(pid, ticks),
                name="g1-dashboard-controller-stop-cleanup",
                daemon=True,
            ).start()
            return self.status()

    def log_tail(self, max_lines: int = 80) -> dict[str, Any]:
        max_lines = max(1, min(200, int(max_lines)))
        try:
            if not self.log_path.exists():
                lines: list[str] = []
            else:
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

    def camera_log_tail(self, max_lines: int = 80) -> dict[str, Any]:
        max_lines = max(1, min(200, int(max_lines)))
        try:
            if not self.camera_log_path.exists():
                lines: list[str] = []
            else:
                with self.camera_log_path.open("rb") as f:
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
        return {"schema": "g1_dashboard.camera_process.v1", "log_path": str(self.camera_log_path), "lines": lines}
