#!/usr/bin/env python3
"""Manage the read-only G1-to-Quest full-body telemetry sender."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

SENDER_BASENAME = "g1_quest_fullbody_sender.py"


def _process_identity(pid: int) -> int | None:
    try:
        stat_raw = Path(f"/proc/{pid}/stat").read_text()
        closing_parenthesis = stat_raw.rfind(")")
        fields = stat_raw[closing_parenthesis + 2:].split()

        if not fields or fields[0] == "Z":
            return None

        start_ticks = int(fields[19])

        command_raw = Path(
            f"/proc/{pid}/cmdline"
        ).read_bytes()

        arguments = [
            item.decode("utf-8", "replace")
            for item in command_raw.split(b"\0")
            if item
        ]

        if not any(
            Path(argument).name == SENDER_BASENAME
            for argument in arguments[1:]
        ):
            return None

        return start_ticks
    except Exception:
        return None


def _matching_pids() -> list[int]:
    result: list[int] = []
    proc = Path("/proc")

    if not proc.is_dir():
        return result

    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue

        pid = int(entry.name)

        if _process_identity(pid) is not None:
            result.append(pid)

    return sorted(set(result))


def _sender_environment(
    python_path: Path,
) -> dict[str, str]:
    env = os.environ.copy()

    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONNOUSERSITE"] = "1"

    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)
    env.pop("CYCLONEDDS_URI", None)

    conda_prefix = python_path.parent.parent
    env["PATH"] = (
        f"{conda_prefix / 'bin'}:"
        f"{env.get('PATH', '')}"
    )
    env["CONDA_PREFIX"] = str(conda_prefix)
    env["CONDA_DEFAULT_ENV"] = conda_prefix.name

    library_entries = [
        entry
        for entry in env.get(
            "LD_LIBRARY_PATH",
            "",
        ).split(":")
        if entry
    ]

    if "/usr/local/lib" not in library_entries:
        library_entries.insert(0, "/usr/local/lib")

    env["LD_LIBRARY_PATH"] = ":".join(
        library_entries
    )

    return env


class FullBodySenderManager:
    def __init__(
        self,
        *,
        python_path: Path,
    ):
        self.python_path = Path(
            python_path
        ).expanduser()

        self.sender_path = Path(
            os.environ.get(
                "G1_DASHBOARD_FULLBODY_SENDER",
                str(
                    Path.home()
                    / SENDER_BASENAME
                ),
            )
        ).expanduser()

        self.state_path = Path(
            os.environ.get(
                "G1_DASHBOARD_FULLBODY_STATE",
                "/tmp/g1_dashboard_fullbody_sender_state.json",
            )
        ).expanduser()

        self.log_path = Path(
            os.environ.get(
                "G1_DASHBOARD_FULLBODY_LOG",
                str(
                    Path.home()
                    / ".local/state/g1_dashboard"
                    / "fullbody_sender.log"
                ),
            )
        ).expanduser()

        self.lock = threading.RLock()
        self.managed: dict[str, Any] | None = None
        self.process = None
        self.last_error: str | None = None
        self.last_exit_code: int | None = None

        self._load_state()

    def _load_state(self) -> None:
        try:
            state = json.loads(
                self.state_path.read_text(
                    encoding="utf-8"
                )
            )

            pid = int(state.get("pid", 0))
            expected_ticks_raw = state.get(
                "start_ticks"
            )
            expected_ticks = (
                int(expected_ticks_raw)
                if expected_ticks_raw is not None
                else None
            )
            actual_ticks = _process_identity(pid)

            if (
                actual_ticks is not None and
                (
                    expected_ticks is None or
                    actual_ticks == expected_ticks
                )
            ):
                self.managed = state
                return
        except Exception:
            pass

        self.managed = None
        self._save_state()

    def _save_state(self) -> None:
        if self.managed is None:
            try:
                self.state_path.unlink(
                    missing_ok=True
                )
            except Exception:
                pass
            return

        self.state_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        temporary = self.state_path.with_suffix(
            ".tmp"
        )
        temporary.write_text(
            json.dumps(
                self.managed,
                indent=2,
                sort_keys=True,
            ),
            encoding="utf-8",
        )
        os.chmod(temporary, 0o600)
        temporary.replace(self.state_path)

    def _current(
        self,
    ) -> tuple[bool, int | None]:
        if self.process is not None:
            return_code = self.process.poll()

            if return_code is not None:
                self.last_exit_code = int(
                    return_code
                )
                self.process = None

        if self.managed is None:
            return False, None

        pid = int(self.managed.get("pid", 0))
        expected_raw = self.managed.get(
            "start_ticks"
        )
        expected = (
            int(expected_raw)
            if expected_raw is not None
            else None
        )
        actual = _process_identity(pid)

        if (
            actual is not None and
            (
                expected is None or
                actual == expected
            )
        ):
            return True, pid

        self.managed = None
        self._save_state()
        return False, None

    def status(self) -> dict[str, Any]:
        with self.lock:
            alive, managed_pid = self._current()

            external_pids = [
                pid
                for pid in _matching_pids()
                if pid != managed_pid
            ]

            if alive:
                state = "RUNNING_MANAGED"
            elif external_pids:
                state = "RUNNING_EXTERNAL"
            else:
                state = "STOPPED"

            return {
                "state": state,
                "managed": alive,
                "pid": (
                    managed_pid
                    if alive
                    else (
                        external_pids[0]
                        if external_pids
                        else None
                    )
                ),
                "sender": str(self.sender_path),
                "python": str(self.python_path),
                "log_path": str(self.log_path),
                "external_pids": external_pids,
                "last_error": self.last_error,
                "last_exit_code": self.last_exit_code,
                "started_now": False,
            }

    def _preflight(
        self,
        env: dict[str, str],
    ) -> str:
        code = (
            "import sys; "
            "from unitree_sdk2py.core.channel "
            "import ChannelFactoryInitialize,"
            "ChannelSubscriber; "
            "from unitree_sdk2py.idl.unitree_hg."
            "msg.dds_ import LowState_; "
            "print('python=' + sys.executable); "
            "print('unitree_sdk2py=OK'); "
            "print('HgLowState=OK')"
        )

        result = subprocess.run(
            [str(self.python_path), "-c", code],
            cwd=str(self.sender_path.parent),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15,
            check=False,
            text=True,
        )

        output = (result.stdout or "").strip()

        if result.returncode != 0:
            raise RuntimeError(
                "full-body sender preflight "
                "failed: " +
                (output or "<no output>")
            )

        return output

    def start(self) -> dict[str, Any]:
        with self.lock:
            alive, _pid = self._current()

            if alive:
                return self.status()

            external = _matching_pids()

            if external:
                raise RuntimeError(
                    "full-body sender already runs "
                    "outside this dashboard "
                    f"(pid {external[0]})"
                )

            if not self.sender_path.is_file():
                raise FileNotFoundError(
                    "sender not found: " +
                    str(self.sender_path)
                )

            if not (
                self.python_path.is_file() and
                os.access(
                    self.python_path,
                    os.X_OK,
                )
            ):
                raise FileNotFoundError(
                    "Python is not executable: " +
                    str(self.python_path)
                )

            env = _sender_environment(
                self.python_path
            )
            preflight = self._preflight(env)

            self.log_path.parent.mkdir(
                parents=True,
                exist_ok=True,
            )

            command = [
                str(self.python_path),
                str(self.sender_path),
            ]

            log = self.log_path.open(
                "ab",
                buffering=0,
            )

            log.write(
                (
                    "\n===== dashboard full-body "
                    f"launch {time.strftime('%Y-%m-%d %H:%M:%S')} "
                    "=====\n"
                    f"command: {' '.join(command)}\n"
                    "destination: "
                    f"{env.get('G1_QUEST_TELEMETRY_IP', '192.168.0.183')}:"
                    f"{env.get('G1_QUEST_TELEMETRY_PORT', '5055')}\n"
                    f"preflight:\n{preflight}\n"
                ).encode("utf-8", "replace")
            )

            try:
                process = subprocess.Popen(
                    command,
                    cwd=str(
                        self.sender_path.parent
                    ),
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env=env,
                    start_new_session=True,
                    close_fds=True,
                )
            finally:
                log.close()

            time.sleep(0.15)
            return_code = process.poll()

            if return_code is not None:
                self.last_exit_code = int(
                    return_code
                )
                raise RuntimeError(
                    "full-body sender exited "
                    f"immediately with code "
                    f"{return_code}; inspect "
                    f"{self.log_path}"
                )

            start_ticks = _process_identity(
                process.pid
            )

            if start_ticks is None:
                process.terminate()
                raise RuntimeError(
                    "could not validate the new "
                    "full-body sender process"
                )

            self.process = process
            self.managed = {
                "pid": process.pid,
                "start_ticks": start_ticks,
                "launched_unix_time_s":
                    time.time(),
                "command": command,
            }
            self.last_error = None
            self.last_exit_code = None
            self._save_state()

            result = self.status()
            result["started_now"] = True
            return result

    def stop(
        self,
        *,
        timeout_s: float = 3.0,
    ) -> dict[str, Any]:
        with self.lock:
            alive, pid = self._current()

            if not alive or pid is None:
                return self.status()

            expected_raw = self.managed.get(
                "start_ticks"
            )
            expected = (
                int(expected_raw)
                if expected_raw is not None
                else None
            )

            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

            deadline = (
                time.monotonic() +
                max(0.5, timeout_s)
            )

            while time.monotonic() < deadline:
                actual = _process_identity(pid)

                if (
                    actual is None or
                    (
                        expected is not None and
                        actual != expected
                    )
                ):
                    break

                if self.process is not None:
                    self.process.poll()

                time.sleep(0.05)

            actual = _process_identity(pid)

            if (
                actual is not None and
                (
                    expected is None or
                    actual == expected
                )
            ):
                self.last_error = (
                    f"sender pid {pid} did not stop "
                    f"within {timeout_s:.1f}s"
                )
                result = self.status()
                result["state"] = "STOP_TIMEOUT"
                return result

            if self.process is not None:
                return_code = self.process.poll()
                if return_code is not None:
                    self.last_exit_code = int(
                        return_code
                    )
                self.process = None

            self.managed = None
            self._save_state()
            return self.status()
