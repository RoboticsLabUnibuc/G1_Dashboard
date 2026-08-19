#!/usr/bin/env python3
"""Read-only G1 motion-state and R3-controller survey (non-blocking V2).

This program creates no DDS publisher. It subscribes asynchronously to:
  - rt/sportmodestate
  - rt/lf/sportmodestate
  - rt/wirelesscontroller

It prints a heartbeat even when no DDS samples are arriving, so network/topic
problems are visible and Ctrl+C remains responsive.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import logging
import math
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any, Iterable, Optional

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)


STOP = threading.Event()


def configure_logging() -> logging.Logger:
    logger = logging.getLogger("g1_motion_state_probe")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)-8s [MotionProbe] %(message)s",
            "%H:%M:%S",
        )
    )
    logger.addHandler(handler)
    logger.propagate = False
    return logger


LOG = configure_logging()


def request_stop(signum: int, frame: object) -> None:
    del signum, frame
    STOP.set()


def resolve_dds_type(
    class_candidates: Iterable[str],
) -> tuple[type, str]:
    module_candidates = (
        "unitree_sdk2py.idl.unitree_go.msg.dds_",
        "unitree_sdk2py.idl.unitree_hg.msg.dds_",
    )

    found_names: list[str] = []
    import_errors: list[str] = []

    for module_name in module_candidates:
        try:
            module = importlib.import_module(module_name)
        except Exception as exc:
            import_errors.append(f"{module_name}: {exc}")
            continue

        for name in dir(module):
            lowered = name.lower()
            if "sport" in lowered or "wireless" in lowered:
                found_names.append(f"{module_name}.{name}")

        for class_name in class_candidates:
            obj = getattr(module, class_name, None)
            if isinstance(obj, type):
                return obj, f"{module_name}.{class_name}"

    detail = "\n  ".join(sorted(set(found_names))) or "<none>"
    errors = "\n  ".join(import_errors) or "<none>"
    raise ImportError(
        "Could not resolve DDS message type. Matching exports:\n"
        f"  {detail}\nImport errors:\n  {errors}"
    )


def public_fields(message: object) -> dict[str, Any]:
    result: dict[str, Any] = {}

    for name in sorted(dir(message)):
        if name.startswith("_"):
            continue

        try:
            value = getattr(message, name)
        except Exception:
            continue

        if callable(value):
            continue

        result[name] = value

    return result


def compact_value(value: Any, max_chars: int = 220) -> str:
    try:
        if isinstance(value, np.ndarray):
            text = np.array2string(
                value,
                precision=4,
                suppress_small=True,
                threshold=20,
            )
        elif isinstance(value, (list, tuple)):
            text = np.array2string(
                np.asarray(value),
                precision=4,
                suppress_small=True,
                threshold=20,
            )
        else:
            text = repr(value)
    except Exception:
        text = repr(value)

    if len(text) > max_chars:
        text = text[: max_chars - 3] + "..."

    return text


def log_field_inventory(label: str, message: object) -> None:
    LOG.info("%s message type: %s", label, type(message))

    fields = public_fields(message)
    if not fields:
        LOG.warning("%s has no visible public data fields.", label)
        return

    for name, value in fields.items():
        LOG.info("%s field %-24s = %s", label, name, compact_value(value))


def first_attr(message: object, names: Iterable[str]) -> Any:
    for name in names:
        if not hasattr(message, name):
            continue

        try:
            return getattr(message, name)
        except Exception:
            pass

    return None


def numeric_array(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None

    try:
        array = np.asarray(value, dtype=float).reshape(-1)
    except Exception:
        return None

    if array.size == 0 or not np.isfinite(array).all():
        return None

    return array


def extract_velocity(message: object) -> tuple[float, float, float]:
    value = first_attr(
        message,
        (
            "velocity",
            "vel",
            "body_velocity",
            "body_vel",
            "linear_velocity",
        ),
    )
    array = numeric_array(value)

    if array is None:
        return math.nan, math.nan, math.nan

    vx = float(array[0]) if array.size >= 1 else math.nan
    vy = float(array[1]) if array.size >= 2 else math.nan
    vz = float(array[2]) if array.size >= 3 else math.nan
    return vx, vy, vz


def extract_yaw_rate(message: object) -> float:
    value = first_attr(
        message,
        (
            "yaw_speed",
            "yawSpeed",
            "yaw_rate",
            "angular_velocity",
            "omega",
        ),
    )
    array = numeric_array(value)

    if array is None:
        return math.nan

    if array.size == 1:
        return float(array[0])

    if array.size >= 3:
        return float(array[2])

    return float(array[-1])


def extract_axis(message: object, names: Iterable[str]) -> float:
    value = first_attr(message, names)
    array = numeric_array(value)

    if array is None:
        return math.nan

    return float(array[0])


def extract_controller(message: object) -> dict[str, Any]:
    return {
        "lx": extract_axis(message, ("lx", "left_x", "leftStickX")),
        "ly": extract_axis(message, ("ly", "left_y", "leftStickY")),
        "rx": extract_axis(message, ("rx", "right_x", "rightStickX")),
        "ry": extract_axis(message, ("ry", "right_y", "rightStickY")),
        "keys": first_attr(message, ("keys", "key", "buttons")),
    }


def finite_or_blank(value: float) -> str:
    return "" if not math.isfinite(value) else f"{value:.9f}"


class LatestSample:
    def __init__(self, label: str) -> None:
        self.label = label
        self._lock = threading.Lock()
        self._message: Optional[object] = None
        self._received_monotonic: Optional[float] = None
        self._count = 0
        self._inventory_done = False
        self._source = ""

    def callback(self, source: str):
        def receive(message: object) -> None:
            inventory_message: Optional[object] = None

            with self._lock:
                self._message = message
                self._received_monotonic = time.monotonic()
                self._count += 1
                self._source = source

                if not self._inventory_done:
                    self._inventory_done = True
                    inventory_message = message

            if inventory_message is not None:
                log_field_inventory(
                    f"{self.label} ({source})",
                    inventory_message,
                )

        return receive

    def snapshot(
        self,
    ) -> tuple[Optional[object], Optional[float], int, str]:
        with self._lock:
            return (
                self._message,
                self._received_monotonic,
                self._count,
                self._source,
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only, non-blocking survey of G1 motion and R3 state."
        )
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Use DDS domain 1.",
    )
    parser.add_argument(
        "--network-interface",
        type=str,
        default=None,
        help="DDS interface for the physical robot.",
    )
    parser.add_argument(
        "--sport-topic",
        action="append",
        default=None,
        help=(
            "Sport-state topic. May be supplied more than once. Defaults to "
            "rt/sportmodestate and rt/lf/sportmodestate."
        ),
    )
    parser.add_argument(
        "--controller-topic",
        type=str,
        default="rt/wirelesscontroller",
    )
    parser.add_argument(
        "--report-hz",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--stationary-linear-mps",
        type=float,
        default=0.05,
    )
    parser.add_argument(
        "--stationary-yaw-rps",
        type=float,
        default=0.08,
    )
    parser.add_argument(
        "--stationary-hold-s",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Zero runs until Ctrl+C.",
    )
    parser.add_argument(
        "--no-data-warning-s",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=str(Path.home() / "g1_motion_state_probe_v2.csv"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.report_hz <= 0:
        raise ValueError("--report-hz must be positive")
    if args.stationary_linear_mps <= 0:
        raise ValueError("--stationary-linear-mps must be positive")
    if args.stationary_yaw_rps <= 0:
        raise ValueError("--stationary-yaw-rps must be positive")
    if args.stationary_hold_s <= 0:
        raise ValueError("--stationary-hold-s must be positive")
    if args.no_data_warning_s <= 0:
        raise ValueError("--no-data-warning-s must be positive")

    sport_topics = args.sport_topic or (
        "rt/sportmodestate",
        "rt/lf/sportmodestate",
    )

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    sport_type, sport_type_name = resolve_dds_type(
        ("SportModeState_", "SportModeState")
    )
    controller_type, controller_type_name = resolve_dds_type(
        ("WirelessController_", "WirelessController")
    )

    LOG.info("G1_MOTION_STATE_PROBE_V2 -- READ ONLY, NON-BLOCKING")
    LOG.info("No DDS command publisher is created.")
    LOG.info("Sport message: %s", sport_type_name)
    LOG.info("Controller message: %s", controller_type_name)

    ChannelFactoryInitialize(
        1 if args.sim else 0,
        networkInterface=args.network_interface,
    )

    sport_latest = LatestSample("SPORT")
    controller_latest = LatestSample("R3")
    subscribers: list[ChannelSubscriber] = []

    try:
        for topic in sport_topics:
            subscriber = ChannelSubscriber(topic, sport_type)
            subscriber.Init(sport_latest.callback(topic), 1)
            subscribers.append(subscriber)
            LOG.info("Subscribed asynchronously: %s", topic)

        controller_subscriber = ChannelSubscriber(
            args.controller_topic,
            controller_type,
        )
        controller_subscriber.Init(
            controller_latest.callback(args.controller_topic),
            1,
        )
        subscribers.append(controller_subscriber)
        LOG.info(
            "Subscribed asynchronously: %s",
            args.controller_topic,
        )

        csv_path = Path(os.path.expanduser(args.csv)).resolve()
        csv_path.parent.mkdir(parents=True, exist_ok=True)

        LOG.info(
            "Mode: %s",
            "SIM DDS domain 1" if args.sim else "REAL DDS domain 0",
        )
        LOG.info("Interface: %s", args.network_interface or "<automatic>")
        LOG.info("CSV: %s", csv_path)
        LOG.info(
            "READY gate: planar<%.3f m/s, |yaw|<%.3f rad/s for %.2f s.",
            args.stationary_linear_mps,
            args.stationary_yaw_rps,
            args.stationary_hold_s,
        )
        LOG.info(
            "No keyboard input is required. Once live values appear, use the "
            "R3 remote for the survey. Press Ctrl+C once to exit."
        )

        stationary_since: Optional[float] = None
        last_ready = False
        start_time = time.monotonic()
        next_report = start_time

        with csv_path.open(
            "w",
            newline="",
            encoding="utf-8",
        ) as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=(
                    "elapsed_s",
                    "sport_source",
                    "sport_age_s",
                    "sport_count",
                    "controller_age_s",
                    "controller_count",
                    "vx_mps",
                    "vy_mps",
                    "vz_mps",
                    "planar_speed_mps",
                    "yaw_rate_rps",
                    "stationary_instant",
                    "stationary_ready",
                    "stationary_elapsed_s",
                    "r3_lx",
                    "r3_ly",
                    "r3_rx",
                    "r3_ry",
                    "r3_keys",
                ),
            )
            writer.writeheader()

            while not STOP.is_set():
                now = time.monotonic()

                if args.duration > 0 and now - start_time >= args.duration:
                    break

                if now < next_report:
                    STOP.wait(min(0.02, next_report - now))
                    continue

                next_report = max(
                    next_report + 1.0 / args.report_hz,
                    now,
                )

                (
                    sport_message,
                    sport_received,
                    sport_count,
                    sport_source,
                ) = sport_latest.snapshot()
                (
                    controller_message,
                    controller_received,
                    controller_count,
                    _,
                ) = controller_latest.snapshot()

                sport_age = (
                    now - sport_received
                    if sport_received is not None
                    else math.inf
                )
                controller_age = (
                    now - controller_received
                    if controller_received is not None
                    else math.inf
                )

                vx = vy = vz = yaw_rate = math.nan
                if sport_message is not None:
                    vx, vy, vz = extract_velocity(sport_message)
                    yaw_rate = extract_yaw_rate(sport_message)

                planar_speed = (
                    math.hypot(vx, vy)
                    if math.isfinite(vx) and math.isfinite(vy)
                    else math.nan
                )

                sport_fresh = (
                    sport_message is not None
                    and sport_age <= args.no_data_warning_s
                )

                stationary_instant = (
                    sport_fresh
                    and math.isfinite(planar_speed)
                    and math.isfinite(yaw_rate)
                    and planar_speed <= args.stationary_linear_mps
                    and abs(yaw_rate) <= args.stationary_yaw_rps
                )

                if stationary_instant:
                    if stationary_since is None:
                        stationary_since = now
                else:
                    stationary_since = None

                stationary_elapsed = (
                    now - stationary_since
                    if stationary_since is not None
                    else 0.0
                )
                stationary_ready = (
                    stationary_since is not None
                    and stationary_elapsed >= args.stationary_hold_s
                )

                controller = (
                    extract_controller(controller_message)
                    if controller_message is not None
                    else {
                        "lx": math.nan,
                        "ly": math.nan,
                        "rx": math.nan,
                        "ry": math.nan,
                        "keys": None,
                    }
                )

                if stationary_ready != last_ready:
                    LOG.info(
                        "Gate transition: %s",
                        (
                            "STATIONARY_READY"
                            if stationary_ready
                            else "MOVING/UNSTABLE"
                        ),
                    )
                    last_ready = stationary_ready

                if sport_message is None and controller_message is None:
                    LOG.warning(
                        "WAITING: no sport-state or R3 DDS sample yet "
                        "(sport=%d, R3=%d). Check eno2/profile, robot power, "
                        "Regular mode, and DDS connectivity.",
                        sport_count,
                        controller_count,
                    )
                elif sport_message is None:
                    LOG.warning(
                        "R3 is live, but no sport-state sample yet "
                        "(R3 count=%d).",
                        controller_count,
                    )
                elif controller_message is None:
                    LOG.warning(
                        "Sport state is live from %s, but no R3 sample yet "
                        "(sport count=%d).",
                        sport_source,
                        sport_count,
                    )
                else:
                    LOG.info(
                        "v=[%+.4f %+.4f %+.4f] m/s planar=%.4f | "
                        "yaw=%+.4f rad/s | gate=%s %.2f/%.2f s | "
                        "R3 lx/ly/rx/ry=%s/%s/%s/%s keys=%s | "
                        "age sport/R3=%.3f/%.3f s",
                        vx,
                        vy,
                        vz,
                        planar_speed,
                        yaw_rate,
                        (
                            "READY"
                            if stationary_ready
                            else (
                                "timing"
                                if stationary_instant
                                else "moving"
                            )
                        ),
                        stationary_elapsed,
                        args.stationary_hold_s,
                        (
                            f"{controller['lx']:+.3f}"
                            if math.isfinite(controller["lx"])
                            else "?"
                        ),
                        (
                            f"{controller['ly']:+.3f}"
                            if math.isfinite(controller["ly"])
                            else "?"
                        ),
                        (
                            f"{controller['rx']:+.3f}"
                            if math.isfinite(controller["rx"])
                            else "?"
                        ),
                        (
                            f"{controller['ry']:+.3f}"
                            if math.isfinite(controller["ry"])
                            else "?"
                        ),
                        compact_value(controller["keys"]),
                        sport_age,
                        controller_age,
                    )

                writer.writerow(
                    {
                        "elapsed_s": f"{now - start_time:.9f}",
                        "sport_source": sport_source,
                        "sport_age_s": (
                            ""
                            if not math.isfinite(sport_age)
                            else f"{sport_age:.9f}"
                        ),
                        "sport_count": sport_count,
                        "controller_age_s": (
                            ""
                            if not math.isfinite(controller_age)
                            else f"{controller_age:.9f}"
                        ),
                        "controller_count": controller_count,
                        "vx_mps": finite_or_blank(vx),
                        "vy_mps": finite_or_blank(vy),
                        "vz_mps": finite_or_blank(vz),
                        "planar_speed_mps": finite_or_blank(planar_speed),
                        "yaw_rate_rps": finite_or_blank(yaw_rate),
                        "stationary_instant": int(stationary_instant),
                        "stationary_ready": int(stationary_ready),
                        "stationary_elapsed_s": (
                            f"{stationary_elapsed:.9f}"
                        ),
                        "r3_lx": finite_or_blank(controller["lx"]),
                        "r3_ly": finite_or_blank(controller["ly"]),
                        "r3_rx": finite_or_blank(controller["rx"]),
                        "r3_ry": finite_or_blank(controller["ry"]),
                        "r3_keys": compact_value(controller["keys"]),
                    }
                )
                csv_file.flush()

        LOG.info("Stopping...")
        LOG.info("Saved CSV: %s", csv_path)
        return 0

    finally:
        for subscriber in subscribers:
            try:
                subscriber.Close()
            except Exception as exc:
                LOG.warning("Subscriber close failed: %s", exc)

        LOG.info("Exited. No command was sent.")


if __name__ == "__main__":
    raise SystemExit(main())
