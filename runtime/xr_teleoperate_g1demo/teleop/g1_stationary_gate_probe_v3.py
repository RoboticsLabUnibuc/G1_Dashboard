#!/usr/bin/env python3
"""Read-only G1 stationary-gate survey using LowState + R3.

Why this probe exists
---------------------
Some G1 Regular-mode firmware does not publish the Go-family
SportModeState topic. G1 does reliably publish hg LowState, which contains:
  - torso IMU angular velocity
  - motor joint velocities

For the locomotion-to-XR handover, a practical full-stop gate is therefore:

  R3 sticks neutral
  AND torso yaw rate small
  AND leg/waist joint velocities small
  AND all conditions remain true for a stable hold interval

This program creates no DDS publisher and sends no robot command.
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
    logger = logging.getLogger("g1_stationary_gate_probe")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()

    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)-8s [StopGate] %(message)s",
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


def resolve_type(
    module_names: Iterable[str],
    class_names: Iterable[str],
) -> tuple[type, str]:
    exports: list[str] = []
    errors: list[str] = []

    for module_name in module_names:
        try:
            module = importlib.import_module(module_name)
        except Exception as exc:
            errors.append(f"{module_name}: {exc}")
            continue

        for name in dir(module):
            lowered = name.lower()
            if "lowstate" in lowered or "wireless" in lowered:
                exports.append(f"{module_name}.{name}")

        for class_name in class_names:
            obj = getattr(module, class_name, None)
            if isinstance(obj, type):
                return obj, f"{module_name}.{class_name}"

    raise ImportError(
        "Could not resolve DDS type.\nExports:\n  "
        + ("\n  ".join(sorted(set(exports))) or "<none>")
        + "\nImport errors:\n  "
        + ("\n  ".join(errors) or "<none>")
    )


def compact_value(value: Any, max_chars: int = 240) -> str:
    try:
        if isinstance(value, np.ndarray):
            text = np.array2string(
                value,
                precision=5,
                suppress_small=True,
                threshold=24,
            )
        elif isinstance(value, (list, tuple)):
            text = np.array2string(
                np.asarray(value),
                precision=5,
                suppress_small=True,
                threshold=24,
            )
        else:
            text = repr(value)
    except Exception:
        text = repr(value)

    if len(text) > max_chars:
        text = text[: max_chars - 3] + "..."

    return text


def public_fields(message: object) -> dict[str, Any]:
    fields: dict[str, Any] = {}

    for name in sorted(dir(message)):
        if name.startswith("_"):
            continue

        try:
            value = getattr(message, name)
        except Exception:
            continue

        if callable(value):
            continue

        fields[name] = value

    return fields


def log_field_inventory(label: str, message: object) -> None:
    LOG.info("%s message type: %s", label, type(message))

    fields = public_fields(message)
    for name, value in fields.items():
        # Motor arrays are large; summarize them rather than dumping all joints.
        if name in ("motor_state", "motorState"):
            try:
                LOG.info(
                    "%s field %-24s = <%d motor states>",
                    label,
                    name,
                    len(value),
                )
            except Exception:
                LOG.info(
                    "%s field %-24s = %s",
                    label,
                    name,
                    compact_value(value),
                )
        else:
            LOG.info(
                "%s field %-24s = %s",
                label,
                name,
                compact_value(value),
            )


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


class LatestSample:
    def __init__(self, label: str) -> None:
        self.label = label
        self._lock = threading.Lock()
        self._message: Optional[object] = None
        self._received: Optional[float] = None
        self._count = 0
        self._source = ""
        self._inventory_done = False

    def callback(self, source: str):
        def receive(message: object) -> None:
            inventory: Optional[object] = None

            with self._lock:
                self._message = message
                self._received = time.monotonic()
                self._count += 1
                self._source = source

                if not self._inventory_done:
                    self._inventory_done = True
                    inventory = message

            if inventory is not None:
                log_field_inventory(
                    f"{self.label} ({source})",
                    inventory,
                )

        return receive

    def snapshot(
        self,
    ) -> tuple[Optional[object], Optional[float], int, str]:
        with self._lock:
            return (
                self._message,
                self._received,
                self._count,
                self._source,
            )


def extract_imu(lowstate: object) -> tuple[float, float, float, float]:
    imu = first_attr(lowstate, ("imu_state", "imuState"))
    if imu is None:
        return math.nan, math.nan, math.nan, math.nan

    gyro = numeric_array(
        first_attr(
            imu,
            (
                "gyroscope",
                "gyro",
                "angular_velocity",
                "angularVelocity",
            ),
        )
    )

    if gyro is None:
        return math.nan, math.nan, math.nan, math.nan

    gx = float(gyro[0]) if gyro.size >= 1 else math.nan
    gy = float(gyro[1]) if gyro.size >= 2 else math.nan
    gz = float(gyro[2]) if gyro.size >= 3 else math.nan
    norm = float(np.linalg.norm(gyro))
    return gx, gy, gz, norm


def extract_joint_dq(
    lowstate: object,
) -> tuple[np.ndarray, int]:
    motors = first_attr(lowstate, ("motor_state", "motorState"))
    if motors is None:
        return np.empty(0, dtype=float), 0

    values: list[float] = []

    try:
        motor_count = len(motors)
    except Exception:
        motor_count = 0

    for motor in motors:
        value = first_attr(motor, ("dq", "velocity"))
        try:
            dq = float(value)
        except Exception:
            dq = math.nan
        values.append(dq)

    return np.asarray(values, dtype=float), motor_count


def finite_metric(values: np.ndarray) -> tuple[float, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return math.nan, math.nan

    max_abs = float(np.max(np.abs(finite)))
    rms = float(np.sqrt(np.mean(np.square(finite))))
    return max_abs, rms


def extract_controller(message: object) -> dict[str, Any]:
    def axis(names: Iterable[str]) -> float:
        value = numeric_array(first_attr(message, names))
        if value is None:
            return math.nan
        return float(value[0])

    return {
        "lx": axis(("lx", "left_x", "leftStickX")),
        "ly": axis(("ly", "left_y", "leftStickY")),
        "rx": axis(("rx", "right_x", "rightStickX")),
        "ry": axis(("ry", "right_y", "rightStickY")),
        "keys": first_attr(message, ("keys", "key", "buttons")),
    }


def finite_or_blank(value: float) -> str:
    return "" if not math.isfinite(value) else f"{value:.9f}"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only G1 full-stop survey using hg LowState and R3."
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
    )
    parser.add_argument(
        "--lowstate-topic",
        action="append",
        default=None,
        help=(
            "LowState topic; repeat for multiple topics. Defaults to "
            "rt/lowstate and rt/lf/lowstate."
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
        "--leg-dq-max-rps",
        type=float,
        default=0.15,
        help=(
            "Provisional maximum absolute leg/waist joint velocity for the "
            "read-only READY gate."
        ),
    )
    parser.add_argument(
        "--yaw-rate-max-rps",
        type=float,
        default=0.08,
        help="Provisional maximum absolute torso yaw rate.",
    )
    parser.add_argument(
        "--r3-deadband",
        type=float,
        default=0.12,
        help=(
            "Maximum absolute R3 axis value considered neutral. The prior "
            "survey observed about -0.051 on neutral rx."
        ),
    )
    parser.add_argument(
        "--stationary-hold-s",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--fresh-lowstate-s",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Zero runs until Ctrl+C.",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=str(Path.home() / "g1_stationary_gate_probe_v3.csv"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.report_hz <= 0:
        raise ValueError("--report-hz must be positive")
    if args.leg_dq_max_rps <= 0:
        raise ValueError("--leg-dq-max-rps must be positive")
    if args.yaw_rate_max_rps <= 0:
        raise ValueError("--yaw-rate-max-rps must be positive")
    if args.r3_deadband <= 0:
        raise ValueError("--r3-deadband must be positive")
    if args.stationary_hold_s <= 0:
        raise ValueError("--stationary-hold-s must be positive")
    if args.fresh_lowstate_s <= 0:
        raise ValueError("--fresh-lowstate-s must be positive")

    lowstate_topics = args.lowstate_topic or (
        "rt/lowstate",
        "rt/lf/lowstate",
    )

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    lowstate_type, lowstate_type_name = resolve_type(
        ("unitree_sdk2py.idl.unitree_hg.msg.dds_",),
        ("LowState_", "LowState"),
    )
    controller_type, controller_type_name = resolve_type(
        ("unitree_sdk2py.idl.unitree_go.msg.dds_",),
        ("WirelessController_", "WirelessController"),
    )

    LOG.info("G1_STATIONARY_GATE_PROBE_V3 -- READ ONLY")
    LOG.info("No DDS command publisher is created.")
    LOG.info("LowState type: %s", lowstate_type_name)
    LOG.info("Controller type: %s", controller_type_name)

    ChannelFactoryInitialize(
        1 if args.sim else 0,
        networkInterface=args.network_interface,
    )

    lowstate_latest = LatestSample("LOWSTATE")
    controller_latest = LatestSample("R3")
    subscribers: list[ChannelSubscriber] = []

    try:
        for topic in lowstate_topics:
            subscriber = ChannelSubscriber(topic, lowstate_type)
            subscriber.Init(lowstate_latest.callback(topic), 1)
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
            "Mode=%s interface=%s",
            "SIM" if args.sim else "REAL",
            args.network_interface or "<automatic>",
        )
        LOG.info(
            "Provisional READY: max|leg/waist dq|<%.3f rad/s, "
            "|gyro z|<%.3f rad/s, max|R3 axis|<%.3f for %.2f s.",
            args.leg_dq_max_rps,
            args.yaw_rate_max_rps,
            args.r3_deadband,
            args.stationary_hold_s,
        )
        LOG.info("CSV: %s", csv_path)
        LOG.info(
            "No key press is needed. Stand 10 s; walk and turn; stop 10 s; "
            "then press Ctrl+C."
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
                    "lowstate_source",
                    "lowstate_age_s",
                    "lowstate_count",
                    "r3_count",
                    "motor_count",
                    "gyro_x_rps",
                    "gyro_y_rps",
                    "gyro_z_rps",
                    "gyro_norm_rps",
                    "leg_waist_dq_max_rps",
                    "leg_waist_dq_rms_rps",
                    "arm_dq_max_rps",
                    "r3_lx",
                    "r3_ly",
                    "r3_rx",
                    "r3_ry",
                    "r3_max_abs",
                    "r3_keys",
                    "lowstate_fresh",
                    "r3_neutral",
                    "motion_quiet",
                    "stationary_ready",
                    "stationary_elapsed_s",
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
                    lowstate,
                    lowstate_received,
                    lowstate_count,
                    lowstate_source,
                ) = lowstate_latest.snapshot()
                (
                    controller,
                    _,
                    controller_count,
                    _,
                ) = controller_latest.snapshot()

                lowstate_age = (
                    now - lowstate_received
                    if lowstate_received is not None
                    else math.inf
                )
                lowstate_fresh = (
                    lowstate is not None
                    and lowstate_age <= args.fresh_lowstate_s
                )

                gx = gy = gz = gyro_norm = math.nan
                leg_dq_max = leg_dq_rms = math.nan
                arm_dq_max = math.nan
                motor_count = 0

                if lowstate is not None:
                    gx, gy, gz, gyro_norm = extract_imu(lowstate)
                    all_dq, motor_count = extract_joint_dq(lowstate)

                    # G1 29-DoF indexing:
                    # 0..11 legs, 12..14 waist, 15..28 arms.
                    leg_waist = all_dq[:15]
                    arms = all_dq[15:29]

                    leg_dq_max, leg_dq_rms = finite_metric(leg_waist)
                    arm_dq_max, _ = finite_metric(arms)

                r3 = (
                    extract_controller(controller)
                    if controller is not None
                    else {
                        "lx": math.nan,
                        "ly": math.nan,
                        "rx": math.nan,
                        "ry": math.nan,
                        "keys": None,
                    }
                )

                axes = np.asarray(
                    [r3["lx"], r3["ly"], r3["rx"], r3["ry"]],
                    dtype=float,
                )
                finite_axes = axes[np.isfinite(axes)]
                r3_max_abs = (
                    float(np.max(np.abs(finite_axes)))
                    if finite_axes.size == 4
                    else math.nan
                )
                r3_neutral = (
                    controller is not None
                    and math.isfinite(r3_max_abs)
                    and r3_max_abs <= args.r3_deadband
                )

                motion_quiet = (
                    lowstate_fresh
                    and math.isfinite(leg_dq_max)
                    and math.isfinite(gz)
                    and leg_dq_max <= args.leg_dq_max_rps
                    and abs(gz) <= args.yaw_rate_max_rps
                )

                stationary_instant = motion_quiet and r3_neutral

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

                if lowstate is None and controller is None:
                    LOG.warning(
                        "WAITING: no hg LowState or R3 sample "
                        "(lowstate=%d, R3=%d).",
                        lowstate_count,
                        controller_count,
                    )
                elif lowstate is None:
                    LOG.warning(
                        "R3 live, but no hg LowState yet "
                        "(R3=%d).",
                        controller_count,
                    )
                else:
                    LOG.info(
                        "leg/waist dq max/rms=%.4f/%.4f rad/s | "
                        "gyro=[%+.4f %+.4f %+.4f] rad/s | "
                        "R3=%s/%s/%s/%s max=%.3f neutral=%s | "
                        "gate=%s %.2f/%.2f s | age=%.3f s",
                        leg_dq_max,
                        leg_dq_rms,
                        gx,
                        gy,
                        gz,
                        (
                            f"{r3['lx']:+.3f}"
                            if math.isfinite(r3["lx"])
                            else "?"
                        ),
                        (
                            f"{r3['ly']:+.3f}"
                            if math.isfinite(r3["ly"])
                            else "?"
                        ),
                        (
                            f"{r3['rx']:+.3f}"
                            if math.isfinite(r3["rx"])
                            else "?"
                        ),
                        (
                            f"{r3['ry']:+.3f}"
                            if math.isfinite(r3["ry"])
                            else "?"
                        ),
                        r3_max_abs,
                        r3_neutral,
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
                        lowstate_age,
                    )

                writer.writerow(
                    {
                        "elapsed_s": f"{now - start_time:.9f}",
                        "lowstate_source": lowstate_source,
                        "lowstate_age_s": (
                            ""
                            if not math.isfinite(lowstate_age)
                            else f"{lowstate_age:.9f}"
                        ),
                        "lowstate_count": lowstate_count,
                        "r3_count": controller_count,
                        "motor_count": motor_count,
                        "gyro_x_rps": finite_or_blank(gx),
                        "gyro_y_rps": finite_or_blank(gy),
                        "gyro_z_rps": finite_or_blank(gz),
                        "gyro_norm_rps": finite_or_blank(gyro_norm),
                        "leg_waist_dq_max_rps": finite_or_blank(leg_dq_max),
                        "leg_waist_dq_rms_rps": finite_or_blank(leg_dq_rms),
                        "arm_dq_max_rps": finite_or_blank(arm_dq_max),
                        "r3_lx": finite_or_blank(r3["lx"]),
                        "r3_ly": finite_or_blank(r3["ly"]),
                        "r3_rx": finite_or_blank(r3["rx"]),
                        "r3_ry": finite_or_blank(r3["ry"]),
                        "r3_max_abs": finite_or_blank(r3_max_abs),
                        "r3_keys": compact_value(r3["keys"]),
                        "lowstate_fresh": int(lowstate_fresh),
                        "r3_neutral": int(r3_neutral),
                        "motion_quiet": int(motion_quiet),
                        "stationary_ready": int(stationary_ready),
                        "stationary_elapsed_s": (
                            f"{stationary_elapsed:.9f}"
                        ),
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
