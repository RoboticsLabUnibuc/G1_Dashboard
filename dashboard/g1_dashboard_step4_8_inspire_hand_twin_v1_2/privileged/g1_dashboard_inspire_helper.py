#!/usr/bin/python3
"""Root-owned lifecycle helper for the Unitree RH56DFX Inspire service.

This helper is intentionally tiny and command-whitelisted.  The dashboard may
invoke only ``start`` and ``stop`` through sudoers.  It never accepts an
executable path, environment value, PID, shell fragment, or arbitrary command.

The installer copies the validated inspire_g1 binary to a root-owned location
next to this helper.  That prevents a passwordless sudo rule from executing a
user-writable binary as root.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

INSTALL_DIR = Path("/usr/local/libexec/g1-dashboard")
BINARY = INSTALL_DIR / "inspire_g1"
BINARY_SHA_FILE = INSTALL_DIR / "inspire_g1.sha256"
STATE_DIR = Path("/run/g1-dashboard")
STATE_FILE = STATE_DIR / "inspire_g1.json"
LOG_DIR = Path("/var/log/g1-dashboard")
LOG_FILE = LOG_DIR / "inspire_g1.log"
SERVICE_NAME = "inspire_g1"


def _proc_start_ticks(pid: int) -> int | None:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        close = raw.rfind(")")
        fields = raw[close + 2 :].split()
        return int(fields[19])
    except Exception:
        return None


def _proc_cmdline(pid: int) -> list[str]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        return [part.decode("utf-8", "replace") for part in raw.split(b"\0") if part]
    except Exception:
        return []


def _alive(pid: int, ticks: int | None = None) -> bool:
    if pid <= 1 or not Path(f"/proc/{pid}").exists():
        return False
    if ticks is not None and _proc_start_ticks(pid) != ticks:
        return False
    return True


def _looks_like_service(pid: int) -> bool:
    args = _proc_cmdline(pid)
    return any(Path(arg).name == SERVICE_NAME for arg in args)


def _matching_pids() -> list[int]:
    out: list[int] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == os.getpid():
            continue
        if _looks_like_service(pid):
            out.append(pid)
    return sorted(set(out))


def _load_state() -> dict | None:
    try:
        data = json.loads(STATE_FILE.read_text())
        pid = int(data.get("pid", 0))
        ticks = data.get("start_ticks")
        ticks = int(ticks) if ticks is not None else None
        if _alive(pid, ticks) and _looks_like_service(pid):
            return data
    except Exception:
        pass
    try:
        STATE_FILE.unlink(missing_ok=True)
    except Exception:
        pass
    return None


def _save_state(data: dict | None) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    os.chmod(STATE_DIR, 0o755)
    if data is None:
        STATE_FILE.unlink(missing_ok=True)
        return
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2, sort_keys=True))
    os.chmod(tmp, 0o644)
    tmp.replace(STATE_FILE)


def status() -> dict:
    managed = _load_state()
    matching = _matching_pids()
    managed_pid = int(managed["pid"]) if managed else None
    external = [pid for pid in matching if pid != managed_pid]
    if managed and external:
        state = "CONFLICT"
    elif managed:
        state = "RUNNING_MANAGED"
    elif external:
        state = "RUNNING_EXTERNAL" if len(external) == 1 else "CONFLICT"
    else:
        state = "STOPPED"
    return {
        "schema": "g1_dashboard.inspire_service.v1",
        "state": state,
        "managed": bool(managed),
        "pid": managed_pid if managed else (external[0] if len(external) == 1 else None),
        "external_pids": external,
        "binary": str(BINARY),
        "binary_exists": BINARY.is_file(),
        "log_path": str(LOG_FILE),
        "can_start": os.geteuid() == 0 and state == "STOPPED" and BINARY.is_file(),
        "can_stop": os.geteuid() == 0 and bool(managed) and not external,
    }


def _require_root() -> None:
    if os.geteuid() != 0:
        raise PermissionError("start/stop must run as root through the installed sudoers rule")


def start() -> dict:
    _require_root()
    st = status()
    if st["state"] == "RUNNING_MANAGED":
        return st
    if st["state"] in {"RUNNING_EXTERNAL", "CONFLICT"}:
        raise RuntimeError(f"refusing to start: Inspire process state is {st['state']}")
    if not BINARY.is_file() or not os.access(BINARY, os.X_OK):
        raise FileNotFoundError(f"installed root-owned Inspire binary not executable: {BINARY}")

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    os.chmod(LOG_DIR, 0o755)
    with LOG_FILE.open("ab", buffering=0) as log:
        log.write(
            f"\n===== dashboard Inspire launch {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n".encode()
        )
        env = {
            "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
            "HOME": "/root",
            "USER": "root",
            "LOGNAME": "root",
            "CYCLONEDDS_URI": "",
            "LD_LIBRARY_PATH": "/usr/local/lib",
        }
        proc = subprocess.Popen(
            [str(BINARY)],
            cwd=str(INSTALL_DIR),
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
            close_fds=True,
        )
    ticks = None
    for _ in range(20):
        ticks = _proc_start_ticks(proc.pid)
        if ticks is not None:
            break
        time.sleep(0.01)
    data = {
        "pid": proc.pid,
        "start_ticks": ticks,
        "launched_unix_time_s": time.time(),
        "binary": str(BINARY),
    }
    _save_state(data)
    time.sleep(0.25)
    rc = proc.poll()
    if rc is not None:
        _save_state(None)
        raise RuntimeError(f"Inspire service exited immediately with code {rc}; inspect {LOG_FILE}")
    return status()


def stop() -> dict:
    _require_root()
    managed = _load_state()
    st = status()
    if managed is None:
        if st["state"] in {"RUNNING_EXTERNAL", "CONFLICT"}:
            raise PermissionError("refusing to stop an Inspire process not launched by the dashboard helper")
        return st
    if st["state"] == "CONFLICT":
        raise PermissionError("refusing to stop while an additional external Inspire process exists")

    pid = int(managed["pid"])
    ticks = managed.get("start_ticks")
    ticks = int(ticks) if ticks is not None else None
    if not _alive(pid, ticks) or not _looks_like_service(pid):
        _save_state(None)
        return status()

    # Manual validated shutdown uses Ctrl+C, so SIGINT is the primary path.
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        _save_state(None)
        return status()

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if not _alive(pid, ticks):
            _save_state(None)
            return status()
        time.sleep(0.05)

    # Bounded fallback: TERM, never KILL.  The helper reports failure if the
    # service still refuses to exit so an operator can investigate explicitly.
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        _save_state(None)
        return status()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if not _alive(pid, ticks):
            _save_state(None)
            return status()
        time.sleep(0.05)
    raise RuntimeError(f"Inspire service pid {pid} did not exit after SIGINT + SIGTERM")


def main() -> int:
    if len(sys.argv) != 2 or sys.argv[1] not in {"status", "start", "stop"}:
        print("usage: g1_dashboard_inspire_helper.py {status|start|stop}", file=sys.stderr)
        return 64
    action = sys.argv[1]
    try:
        result = status() if action == "status" else start() if action == "start" else stop()
        print(json.dumps(result, separators=(",", ":")))
        return 0
    except PermissionError as exc:
        print(json.dumps({"ok": False, "error": str(exc)}))
        return 77
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
