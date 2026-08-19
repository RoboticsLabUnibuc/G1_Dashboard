#!/usr/bin/env python3
"""Read-only PC2 + Unitree service/base-sensing monitor for the G1 dashboard.

Architecture:
- Samples Linux host health from /proc and /sys using only the stdlib.
- Optionally queries Unitree RobotStateClient ServiceList/API versions.
- Optionally subscribes read-only to G1 torso IMU (rt/secondary_imu).
- Best-effort read-only odometry subscriber for rt/odommodestate + lf fallback.
- Publishes newest diagnostics to localhost UDP only.
- Creates NO robot command publishers and never calls ServiceSwitch.
"""
from __future__ import annotations

import argparse
import fcntl
import importlib
import json
import multiprocessing as mp
import os
import queue as queue_mod
import shutil
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Any, Iterable

SCHEMA = "g1_dashboard.system.v1"
VERSION = "g1_dashboard_system_monitor.v1.1.2-dds-startup-serialization"
DEFAULT_DEST = ("127.0.0.1", 8766)
SIOCGIFADDR = 0x8915


def read_text(path: str | Path, default: str = "") -> str:
    try:
        return Path(path).read_text().strip()
    except Exception:
        return default


def read_int(path: str | Path, default: int | None = None) -> int | None:
    try:
        return int(read_text(path))
    except Exception:
        return default


def meminfo() -> dict[str, int]:
    out: dict[str, int] = {}
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if ":" not in line:
                continue
            key, rest = line.split(":", 1)
            parts = rest.strip().split()
            if not parts:
                continue
            value = int(parts[0])
            if len(parts) > 1 and parts[1].lower() == "kb":
                value *= 1024
            out[key] = value
    except Exception:
        pass
    return out


def cpu_times() -> tuple[int, int] | None:
    try:
        first = Path("/proc/stat").read_text().splitlines()[0].split()
        if not first or first[0] != "cpu":
            return None
        vals = [int(x) for x in first[1:]]
        total = sum(vals)
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        return total, idle
    except Exception:
        return None


def iface_ipv4(name: str) -> str | None:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        packed = struct.pack("256s", name[:15].encode("utf-8"))
        res = fcntl.ioctl(s.fileno(), SIOCGIFADDR, packed)
        return socket.inet_ntoa(res[20:24])
    except Exception:
        return None


def thermal_snapshot() -> dict[str, Any]:
    zones = []
    for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        raw = read_int(zone / "temp")
        if raw is None:
            continue
        c = raw / 1000.0 if abs(raw) > 1000 else float(raw)
        zones.append({"name": read_text(zone / "type", zone.name), "temp_c": c})
    zones.sort(key=lambda z: z["temp_c"], reverse=True)
    return {"max_c": zones[0]["temp_c"] if zones else None, "zones": zones[:8]}


def tcp_open(port: int, host: str = "127.0.0.1", timeout: float = 0.05) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        return s.connect_ex((host, int(port))) == 0
    except Exception:
        return False
    finally:
        s.close()


def process_counts() -> dict[str, int]:
    patterns = {
        "controller": "g1_locomotion_xr_handover_live",
        "teleimager": "teleimager",
        "inspire": "inspire_g1",
        "dashboard_bridge": "g1_dashboard_bridge.py",
    }
    counts = {k: 0 for k in patterns}
    try:
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "ignore")
            except Exception:
                continue
            for key, needle in patterns.items():
                if needle in raw:
                    counts[key] += 1
    except Exception:
        pass
    return counts


def _value(obj: Any, name: str, default: Any = None) -> Any:
    try:
        value = getattr(obj, name)
        return value() if callable(value) else value
    except Exception:
        return default


def _vec(obj: Any, names: Iterable[str], length: int) -> list[float] | None:
    for name in names:
        raw = _value(obj, name, None)
        if raw is None:
            continue
        try:
            values = list(raw)
            if len(values) >= length:
                return [float(values[i]) for i in range(length)]
        except Exception:
            continue
    return None


def _scalar(obj: Any, names: Iterable[str]) -> float | None:
    for name in names:
        raw = _value(obj, name, None)
        if raw is None:
            continue
        try:
            return float(raw)
        except Exception:
            continue
    return None


class UnitreeChannelContext:
    """Initialize Unitree's global DDS channel factory once for this monitor."""

    def __init__(self, interface: str, enabled: bool) -> None:
        self.interface = interface
        self.enabled = enabled
        self.available = False
        self.error: str | None = "disabled" if not enabled else "not initialized"
        self.channel_module: Any = None
        # CycloneDDS/Unitree topic and RPC entity construction is serialized
        # inside this process. The Python binding can otherwise attempt complex
        # TypeObject construction concurrently from different threads.
        self.entity_lock = threading.RLock()

    def initialize(self) -> None:
        if not self.enabled:
            return
        try:
            core = importlib.import_module("unitree_sdk2py.core.channel")
            init = getattr(core, "ChannelFactoryInitialize")
            init(0, self.interface)
            self.channel_module = core
            self.available = True
            self.error = None
        except Exception as exc:
            self.available = False
            self.error = f"Unitree channel init failed: {type(exc).__name__}: {exc}"


class RobotStateProbe:
    """Optional read-only Unitree RobotStateClient worker.

    DDS/RPC entity construction is performed synchronously before any base
    sensing Topic is created. This avoids concurrent CycloneDDS TypeObject
    construction in the Python binding. Only read operations are used;
    ServiceSwitch is deliberately never invoked by this monitor.
    """

    CANDIDATES = (
        "unitree_sdk2py.g1.robot_state.robot_state_client",
        "unitree_sdk2py.go2.robot_state.robot_state_client",
        "unitree_sdk2py.b2.robot_state.robot_state_client",
    )

    def __init__(self, context: UnitreeChannelContext, mode: str) -> None:
        self.context = context
        self.mode = mode
        self._lock = threading.Lock()
        self._snapshot: dict[str, Any] = {
            "enabled": mode != "off",
            "available": False,
            "module": None,
            "client_api_version": None,
            "server_api_version": None,
            "api_version_match": None,
            "service_count": 0,
            "services": [],
            "error": "disabled" if mode == "off" else "not initialized",
            "updated_unix_time_s": None,
        }
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._client: Any = None
        self._module: str | None = None

    def initialize(self) -> bool:
        """Construct and validate RobotStateClient synchronously.

        main() calls this before BaseSensingProbe.start(), so RobotState RPC
        topic construction cannot race SportModeState_/IMU Topic creation.
        """
        if self.mode == "off":
            return True
        if not self.context.available:
            self._set(error=self.context.error or "Unitree DDS unavailable")
            return False

        module = None
        client_cls = None
        import_errors = []
        for name in self.CANDIDATES:
            try:
                m = importlib.import_module(name)
                cls = getattr(m, "RobotStateClient")
                module, client_cls = name, cls
                break
            except Exception as exc:
                import_errors.append(f"{name}: {exc}")
        if client_cls is None:
            self._set(error="RobotStateClient import failed: " + " | ".join(import_errors))
            return False

        try:
            # Topic/request-response entity creation must not overlap with the
            # base-sensing subscriber setup in another thread.
            with self.context.entity_lock:
                client = client_cls()
                if hasattr(client, "SetTimeout"):
                    client.SetTimeout(1.0)
                client.Init()
            self._client = client
            self._module = module
            # Validate one complete read before any odometry Topic is attempted.
            self._poll_once()
            return bool(self.snapshot().get("available"))
        except Exception as exc:
            self._set(module=module, error=f"RobotStateClient init failed: {exc}")
            return False

    def start(self) -> None:
        if self.mode == "off" or self._client is None:
            return
        self._thread = threading.Thread(target=self._run, name="unitree-robot-state-readonly", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._snapshot))

    def _set(self, **kwargs: Any) -> None:
        with self._lock:
            self._snapshot.update(kwargs)
            self._snapshot["updated_unix_time_s"] = time.time()

    @staticmethod
    def _normalize_server_version(value: Any) -> tuple[int | None, str | None]:
        if isinstance(value, tuple) and len(value) >= 2:
            try:
                code = int(value[0])
            except Exception:
                code = None
            return code, None if value[1] is None else str(value[1])
        return 0, None if value is None else str(value)

    def _poll_once(self) -> None:
        client = self._client
        module = self._module
        if client is None:
            return
        client_version = client.GetApiVersion() if hasattr(client, "GetApiVersion") else None
        server_code, server_version = self._normalize_server_version(
            client.GetServerApiVersion() if hasattr(client, "GetServerApiVersion") else None
        )
        code, services = client.ServiceList()
        if int(code) != 0:
            raise RuntimeError(f"ServiceList returned code {code}")
        normalized = []
        for svc in services or []:
            raw_status = int(getattr(svc, "status", -1))
            # Unitree RobotState uses 0 = service ON, 1 = service OFF.
            normalized.append({
                "name": str(getattr(svc, "name", "?")),
                "status": raw_status,
                "enabled": True if raw_status == 0 else False if raw_status == 1 else None,
                "protect": bool(getattr(svc, "protect", False)),
            })
        normalized.sort(key=lambda x: x["name"])
        match = None
        if client_version is not None and server_version is not None and server_code in (None, 0):
            match = str(client_version) == str(server_version)
        self._set(
            available=True,
            module=module,
            client_api_version=None if client_version is None else str(client_version),
            server_api_version=server_version,
            api_version_match=match,
            service_count=len(normalized),
            services=normalized,
            error=None if server_code in (None, 0) else f"GetServerApiVersion code {server_code}",
        )

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._poll_once()
            except Exception as exc:
                # Keep the last known service inventory visible if a transient
                # RPC read fails; mark only the current API availability.
                self._set(available=False, module=self._module, error=str(exc))
            self._stop.wait(5.0)


def _put_latest_mp(q: Any, item: dict[str, Any]) -> None:
    """Best-effort latest-only multiprocessing queue write; never block DDS."""
    try:
        q.put_nowait(item)
        return
    except queue_mod.Full:
        pass
    except Exception:
        return
    try:
        q.get_nowait()
    except Exception:
        pass
    try:
        q.put_nowait(item)
    except Exception:
        pass


def _odometry_worker_main(interface: str, out_q: Any, stop_event: Any) -> None:
    """Isolated read-only odometry DDS process.

    SportModeState_ is a comparatively complex nested IDL type. Keeping its
    Topic/TypeObject construction in a separate process prevents any Python
    CycloneDDS setup failure from affecting RobotStateClient or torso IMU.
    """
    subscribers: list[Any] = []
    count = 0

    def callback(msg: Any, topic: str) -> None:
        nonlocal count
        count += 1
        _put_latest_mp(out_q, {
            "kind": "sample",
            "available": True,
            "topic": topic,
            "position_m": _vec(msg, ("position", "pos"), 3),
            "velocity_mps": _vec(msg, ("velocity", "vel"), 3),
            "yaw_speed_rps": _scalar(msg, ("yaw_speed", "yawSpeed", "yaw_rate")),
            "sample_count": count,
            "received_monotonic": time.monotonic(),
            "error": None,
        })

    try:
        core = importlib.import_module("unitree_sdk2py.core.channel")
        init = getattr(core, "ChannelFactoryInitialize")
        ChannelSubscriber = getattr(core, "ChannelSubscriber")
        init(0, interface)
        dds_go = importlib.import_module("unitree_sdk2py.idl.unitree_go.msg.dds_")
        odom_cls = getattr(dds_go, "SportModeState_")
        for topic in ("rt/odommodestate", "rt/lf/odommodestate"):
            sub = ChannelSubscriber(topic, odom_cls)
            sub.Init(lambda msg, t=topic: callback(msg, t), 0)
            subscribers.append(sub)
        _put_latest_mp(out_q, {
            "kind": "ready",
            "available": False,
            "topic": None,
            "sample_count": 0,
            "error": None,
        })
        while not stop_event.wait(0.2):
            pass
    except Exception as exc:
        detail = f"{type(exc).__name__}: {exc}"
        if "Failed to encode union" in detail or "TypeObject" in detail:
            compact = (
                "Odometry Python DDS type setup failed (SportModeState_ TypeObject). "
                "RobotState and IMU are unaffected because odometry is process-isolated."
            )
        else:
            compact = f"odometry subscribe unavailable: {detail}"
        print(f"[odometry-worker] setup failed: {detail}", flush=True)
        _put_latest_mp(out_q, {
            "kind": "error",
            "available": False,
            "topic": None,
            "sample_count": 0,
            "error": compact,
            "error_detail": detail,
        })
        while not stop_event.wait(0.5):
            pass
    finally:
        for sub in subscribers:
            try:
                sub.Close()
            except Exception:
                pass


class BaseSensingProbe:
    """Read-only G1 torso IMU + best-effort odometry DDS subscribers."""

    def __init__(self, context: UnitreeChannelContext, mode: str) -> None:
        self.context = context
        self.mode = mode
        self._lock = threading.Lock()
        self._imu_received_monotonic: float | None = None
        self._odom_received_monotonic: float | None = None
        self._imu_count = 0
        self._odom_count = 0
        self._imu: dict[str, Any] = {
            "available": False,
            "topic": "rt/secondary_imu",
            "quaternion_wxyz": None,
            "rpy_rad": None,
            "gyroscope_rps": None,
            "accelerometer_mps2": None,
            "temperature_c": None,
            "error": "disabled" if mode == "off" else "not initialized",
        }
        self._odom: dict[str, Any] = {
            "available": False,
            "topic": None,
            "position_m": None,
            "velocity_mps": None,
            "yaw_speed_rps": None,
            "error": "disabled" if mode == "off" else "not initialized",
        }
        self._subscribers: list[Any] = []
        self._mp_ctx = mp.get_context("spawn")
        self._odom_queue: Any = None
        self._odom_stop: Any = None
        self._odom_process: Any = None

    def start(self) -> None:
        if self.mode == "off":
            return
        if not self.context.available:
            with self._lock:
                self._imu["error"] = self.context.error or "Unitree DDS unavailable"
                self._odom["error"] = self.context.error or "Unitree DDS unavailable"
            return
        try:
            ChannelSubscriber = getattr(self.context.channel_module, "ChannelSubscriber")
        except Exception as exc:
            with self._lock:
                self._imu["error"] = f"ChannelSubscriber unavailable: {exc}"
                self._odom["error"] = f"ChannelSubscriber unavailable: {exc}"
            return

        # Official G1 SDK example: rt/secondary_imu uses unitree_hg IMUState_.
        try:
            dds_hg = importlib.import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
            imu_cls = getattr(dds_hg, "IMUState_")
            with self.context.entity_lock:
                sub = ChannelSubscriber("rt/secondary_imu", imu_cls)
                sub.Init(self._imu_callback, 0)
            self._subscribers.append(sub)
            with self._lock:
                self._imu["error"] = None
        except Exception as exc:
            with self._lock:
                self._imu["error"] = f"secondary_imu subscribe failed: {type(exc).__name__}: {exc}"

        # Odometry uses a complex nested SportModeState_ type. Construct it in
        # a spawned process with its own ChannelFactory/DomainParticipant so a
        # TypeObject failure cannot poison RobotStateClient or the torso IMU.
        try:
            self._odom_queue = self._mp_ctx.Queue(maxsize=1)
            self._odom_stop = self._mp_ctx.Event()
            self._odom_process = self._mp_ctx.Process(
                target=_odometry_worker_main,
                args=(self.context.interface, self._odom_queue, self._odom_stop),
                name="g1-dashboard-odometry-readonly",
                daemon=True,
            )
            self._odom_process.start()
            with self._lock:
                self._odom["error"] = None
        except Exception as exc:
            with self._lock:
                self._odom["error"] = f"odometry worker start failed: {type(exc).__name__}: {exc}"

    def stop(self) -> None:
        if self._odom_stop is not None:
            try:
                self._odom_stop.set()
            except Exception:
                pass
        if self._odom_process is not None:
            try:
                self._odom_process.join(timeout=1.0)
                if self._odom_process.is_alive():
                    self._odom_process.terminate()
                    self._odom_process.join(timeout=0.5)
            except Exception:
                pass
        for sub in self._subscribers:
            try:
                sub.Close()
            except Exception:
                pass
        self._subscribers.clear()

    def _imu_callback(self, msg: Any) -> None:
        now = time.monotonic()
        quaternion = _vec(msg, ("quaternion",), 4)
        gyroscope = _vec(msg, ("gyroscope",), 3)
        accelerometer = _vec(msg, ("accelerometer",), 3)
        rpy = _vec(msg, ("rpy",), 3)
        temperature = _scalar(msg, ("temperature",))
        with self._lock:
            self._imu_count += 1
            self._imu_received_monotonic = now
            self._imu.update({
                "available": True,
                "quaternion_wxyz": quaternion,
                "rpy_rad": rpy,
                "gyroscope_rps": gyroscope,
                "accelerometer_mps2": accelerometer,
                "temperature_c": temperature,
                "error": None,
            })

    def _drain_odometry_worker(self) -> None:
        if self._odom_queue is None:
            return
        newest = None
        while True:
            try:
                newest = self._odom_queue.get_nowait()
            except queue_mod.Empty:
                break
            except Exception:
                break
        if not isinstance(newest, dict):
            return
        kind = newest.get("kind")
        with self._lock:
            if kind == "sample":
                self._odom_count = int(newest.get("sample_count") or self._odom_count)
                self._odom_received_monotonic = newest.get("received_monotonic")
                self._odom.update({
                    "available": True,
                    "topic": newest.get("topic"),
                    "position_m": newest.get("position_m"),
                    "velocity_mps": newest.get("velocity_mps"),
                    "yaw_speed_rps": newest.get("yaw_speed_rps"),
                    "error": None,
                })
            elif kind == "error":
                self._odom.update({
                    "available": False,
                    "topic": None,
                    "error": newest.get("error"),
                    "error_detail": newest.get("error_detail"),
                })
            elif kind == "ready":
                self._odom["error"] = None

    def snapshot(self) -> dict[str, Any]:
        self._drain_odometry_worker()
        now = time.monotonic()
        with self._lock:
            imu = json.loads(json.dumps(self._imu))
            odom = json.loads(json.dumps(self._odom))
            imu_age = None if self._imu_received_monotonic is None else max(0.0, now - self._imu_received_monotonic)
            odom_age = None if self._odom_received_monotonic is None else max(0.0, now - self._odom_received_monotonic)
            imu_count = self._imu_count
            odom_count = self._odom_count
        imu["age_s"] = imu_age
        imu["sample_count"] = imu_count
        odom["age_s"] = odom_age
        odom["sample_count"] = odom_count
        return {
            "enabled": self.mode != "off",
            "channel_available": self.context.available,
            "channel_error": self.context.error,
            "imu": imu,
            "odometry": odom,
        }


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Read-only G1 dashboard PC2/system/base-sensing monitor")
    p.add_argument("--network-interface", default="enP8p1s0")
    p.add_argument("--udp-host", default=DEFAULT_DEST[0])
    p.add_argument("--udp-port", type=int, default=DEFAULT_DEST[1])
    p.add_argument("--hz", type=float, default=5.0)
    p.add_argument(
        "--unitree-services",
        choices=("auto", "on", "off"),
        default="auto",
        help="Read Unitree RobotState service list/API versions; never switches services",
    )
    p.add_argument(
        "--base-sensing",
        choices=("auto", "on", "off"),
        default="auto",
        help="Subscribe read-only to secondary IMU and best-effort odometry topics",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.udp_host not in ("127.0.0.1", "localhost"):
        raise SystemExit("Refusing non-loopback destination. Keep --udp-host=127.0.0.1.")
    if args.hz <= 0 or args.hz > 10:
        raise SystemExit("--hz must be >0 and <=10")
    if not (1 <= args.udp_port <= 65535):
        raise SystemExit("--udp-port must be in [1, 65535]")

    need_unitree = args.unitree_services != "off" or args.base_sensing != "off"
    context = UnitreeChannelContext(args.network_interface, need_unitree)
    context.initialize()

    robot_probe = RobotStateProbe(context, args.unitree_services)
    base_probe = BaseSensingProbe(context, args.base_sensing)

    # IMPORTANT: create/validate RobotState RPC entities before creating the
    # secondary-IMU Topic. Odometry is spawned in its own process/participant,
    # so its complex SportModeState_ TypeObject setup cannot affect RobotState.
    robot_probe.initialize()
    base_probe.start()
    # Only after all read-only DDS entities are constructed do we start the
    # periodic RobotState RPC polling thread.
    robot_probe.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", args.udp_port)
    period = 1.0 / args.hz
    sequence = 0
    prev_cpu = cpu_times()

    print(f"{VERSION} READ-ONLY")
    print(f"System output    : udp://127.0.0.1:{args.udp_port} @ {args.hz:.1f} Hz")
    print(f"Network interface: {args.network_interface}")
    print(f"Unitree channel  : {'ready' if context.available else 'unavailable'}{'' if not context.error else ' · ' + context.error}")
    print(f"Unitree services : {args.unitree_services} (ServiceList/API versions only; NO ServiceSwitch)")
    print(f"Base sensing     : {args.base_sensing} (secondary_imu + odometry subscribers; READ-ONLY)")

    try:
        while True:
            started = time.monotonic()
            cur_cpu = cpu_times()
            cpu_pct = None
            if prev_cpu and cur_cpu:
                dt = cur_cpu[0] - prev_cpu[0]
                di = cur_cpu[1] - prev_cpu[1]
                if dt > 0:
                    cpu_pct = max(0.0, min(100.0, 100.0 * (dt - di) / dt))
            prev_cpu = cur_cpu

            mi = meminfo()
            mem_total = mi.get("MemTotal")
            mem_avail = mi.get("MemAvailable")
            mem_used_pct = None
            if mem_total and mem_avail is not None:
                mem_used_pct = 100.0 * (mem_total - mem_avail) / mem_total

            try:
                disk = shutil.disk_usage("/")
                disk_obj = {
                    "total_bytes": disk.total,
                    "free_bytes": disk.free,
                    "used_pct": 100.0 * disk.used / disk.total if disk.total else None,
                }
            except Exception:
                disk_obj = {"total_bytes": None, "free_bytes": None, "used_pct": None}

            try:
                load1, load5, load15 = os.getloadavg()
            except Exception:
                load1 = load5 = load15 = None

            iface = args.network_interface
            net_base = Path("/sys/class/net") / iface
            net_obj = {
                "interface": iface,
                "ipv4": iface_ipv4(iface),
                "operstate": read_text(net_base / "operstate", "unknown"),
                "carrier": read_int(net_base / "carrier"),
                "rx_bytes": read_int(net_base / "statistics/rx_bytes"),
                "tx_bytes": read_int(net_base / "statistics/tx_bytes"),
            }

            try:
                uptime_s = float(read_text("/proc/uptime").split()[0])
            except Exception:
                uptime_s = None

            packet = {
                "schema": SCHEMA,
                "version": VERSION,
                "sequence": sequence,
                "unix_time_s": time.time(),
                "host": {
                    "hostname": socket.gethostname(),
                    "uptime_s": uptime_s,
                    "cpu": {"used_pct": cpu_pct, "load_1m": load1, "load_5m": load5, "load_15m": load15},
                    "memory": {"total_bytes": mem_total, "available_bytes": mem_avail, "used_pct": mem_used_pct},
                    "disk_root": disk_obj,
                    "thermal": thermal_snapshot(),
                    "network": net_obj,
                },
                "endpoints": {
                    "televuer_8012": tcp_open(8012),
                    "camera_60001": tcp_open(60001),
                    "dashboard_8080": tcp_open(8080),
                },
                "processes": process_counts(),
                "robot_state_api": robot_probe.snapshot(),
                "base_sensing": base_probe.snapshot(),
            }
            sequence += 1
            payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
            sock.sendto(payload, dest)
            elapsed = time.monotonic() - started
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        print("\nStopping system monitor...")
    finally:
        base_probe.stop()
        robot_probe.stop()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
