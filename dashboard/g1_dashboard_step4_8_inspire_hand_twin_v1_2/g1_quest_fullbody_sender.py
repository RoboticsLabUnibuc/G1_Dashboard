#!/usr/bin/env python3

import json
import math
import os
import signal
import socket
import threading
import time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowState_ as HgLowState,
)


# ============================================================
# CONFIGURATION
# ============================================================

QUEST_IP = os.environ.get(
    "G1_QUEST_TELEMETRY_IP",
    "192.168.0.183",
)
QUEST_PORT = int(os.environ.get(
    "G1_QUEST_TELEMETRY_PORT",
    "5055",
))

NETWORK_INTERFACE = os.environ.get(
    "G1_QUEST_TELEMETRY_INTERFACE",
    "enP8p1s0",
)

SEND_HZ = float(os.environ.get(
    "G1_QUEST_TELEMETRY_HZ",
    "30",
))
NUM_JOINTS = 29

DASHBOARD_HOST = os.environ.get(
    "G1_DASHBOARD_ROBOT_TELEMETRY_HOST",
    "127.0.0.1",
)
DASHBOARD_PORT = int(os.environ.get(
    "G1_DASHBOARD_ROBOT_TELEMETRY_PORT",
    "8768",
))
DASHBOARD_SCHEMA = "g1_dashboard.robot_telemetry.v1"

JOINT_NAMES = (
    "L_hip_pitch", "L_hip_roll", "L_hip_yaw",
    "L_knee", "L_ankle_pitch", "L_ankle_roll",
    "R_hip_pitch", "R_hip_roll", "R_hip_yaw",
    "R_knee", "R_ankle_pitch", "R_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "L_shoulder_pitch", "L_shoulder_roll",
    "L_shoulder_yaw", "L_elbow", "L_wrist_roll",
    "L_wrist_pitch", "L_wrist_yaw",
    "R_shoulder_pitch", "R_shoulder_roll",
    "R_shoulder_yaw", "R_elbow", "R_wrist_roll",
    "R_wrist_pitch", "R_wrist_yaw",
)

if not (1 <= QUEST_PORT <= 65535):
    raise SystemExit("invalid Quest telemetry UDP port")
if not (1.0 <= SEND_HZ <= 120.0):
    raise SystemExit("invalid Quest telemetry send rate")
if DASHBOARD_HOST not in ("127.0.0.1", "localhost"):
    raise SystemExit(
        "dashboard robot telemetry must remain loopback-only"
    )
if not (1 <= DASHBOARD_PORT <= 65535):
    raise SystemExit(
        "invalid dashboard robot telemetry UDP port"
    )


# ============================================================
# SHARED LOWSTATE DATA
# ============================================================

lock = threading.Lock()

# The DDS callback must remain constant-time. Parsing every high-rate
# LowState sample here can make the reader fall behind during controller
# startup. The 30 Hz sender loop parses only the newest retained sample.
latest_lowstate = None
latest_recv_monotonic = 0.0
latest_recv_unix = 0.0
latest_callback_count = 0

stop_event = threading.Event()


def request_stop(_signum, _frame):
    stop_event.set()


# ============================================================
# DDS CALLBACK
# ============================================================

def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def temperature_pair(motor_state):
    try:
        values = list(motor_state.temperature)
    except Exception:
        return [None, None]

    first = (
        finite_float(values[0])
        if len(values) >= 1
        else None
    )
    second = (
        finite_float(values[1])
        if len(values) >= 2
        else None
    )
    return [first, second]


def parse_lowstate(msg):
    motors = [
        msg.motor_state[i]
        for i in range(NUM_JOINTS)
    ]

    q = [
        finite_float(motor.q)
        for motor in motors
    ]
    dq = [
        finite_float(motor.dq)
        for motor in motors
    ]
    tau_est = [
        finite_float(
            getattr(motor, "tau_est", None)
        )
        for motor in motors
    ]
    temperatures = [
        temperature_pair(motor)
        for motor in motors
    ]

    motor_state = []
    for motor in motors:
        try:
            motor_state.append(
                int(motor.motorstate)
            )
        except Exception:
            motor_state.append(None)

    if any(value is None for value in q):
        raise ValueError(
            "one or more joint positions are non-finite"
        )

    return (
        q,
        dq,
        tau_est,
        temperatures,
        motor_state,
        int(msg.tick),
        int(msg.mode_pr),
        int(msg.mode_machine),
    )


def lowstate_callback(msg):
    global latest_lowstate
    global latest_recv_monotonic
    global latest_recv_unix
    global latest_callback_count

    received_monotonic = time.monotonic()
    received_unix = time.time()

    with lock:
        latest_lowstate = msg
        latest_recv_monotonic = received_monotonic
        latest_recv_unix = received_unix
        latest_callback_count += 1


# ============================================================
# MAIN
# ============================================================

def main():
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    print("==========================================")
    print(" G1 -> QUEST + DASHBOARD FULL-BODY TELEMETRY")
    print("==========================================")
    print()

    print(
        f"Quest destination : "
        f"{QUEST_IP}:{QUEST_PORT}"
    )

    print(
        f"Dashboard stream  : "
        f"{DASHBOARD_HOST}:{DASHBOARD_PORT}"
    )

    print(
        f"DDS interface     : "
        f"{NETWORK_INTERFACE}"
    )

    print(
        f"Send rate         : "
        f"{SEND_HZ:.1f} Hz"
    )

    print(
        f"Joint count       : "
        f"{NUM_JOINTS}"
    )

    print()
    print(
        "READ ONLY: no robot command publisher "
        "is created."
    )
    print()

    # --------------------------------------------------------
    # DDS
    # --------------------------------------------------------

    ChannelFactoryInitialize(
        0,
        networkInterface=NETWORK_INTERFACE,
    )

    subscriber = ChannelSubscriber(
        "rt/lowstate",
        HgLowState,
    )

    # queueLen = 0 executes the constant-time callback directly on the
    # DDS reader notification. Heavy snapshot parsing stays in the 30 Hz
    # sender loop so controller startup cannot build a parsing backlog.
    subscriber.Init(
        lowstate_callback,
        0,
    )

    # --------------------------------------------------------
    # UDP
    # --------------------------------------------------------

    sock = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )

    period = 1.0 / SEND_HZ
    seq = 0

    print("Waiting for rt/lowstate...")

    try:
        while not stop_event.is_set():
            loop_start = time.monotonic()

            with lock:
                sample = latest_lowstate
                recv_monotonic = (
                    latest_recv_monotonic
                )
                sample_unix_time = (
                    latest_recv_unix
                )
                callback_count = (
                    latest_callback_count
                )

            q = None

            if sample is not None:
                try:
                    (
                        q,
                        dq,
                        tau_est,
                        temperatures,
                        motor_state,
                        tick,
                        mode_pr,
                        mode_machine,
                    ) = parse_lowstate(sample)
                except Exception as exc:
                    print(
                        "[WARN] Failed to parse "
                        "LowState snapshot: "
                        f"{exc}"
                    )

            if q is not None:
                sample_age = max(
                    0.0,
                    time.monotonic()
                    - recv_monotonic,
                )

                # ------------------------------------------------
                # Packet format:
                #
                # G1Q
                # | seq
                # | tick
                # | mode_pr
                # | q0
                # | ...
                # | q28
                # | age
                #
                # mode_machine is currently diagnostic only.
                # ------------------------------------------------

                fields = [
                    "G1Q",
                    str(seq),
                    str(tick),
                    str(mode_pr),
                ]

                fields.extend(
                    f"{value:.6f}"
                    for value in q
                )

                fields.append(
                    f"{sample_age:.6f}"
                )

                message = "|".join(fields)

                sock.sendto(
                    message.encode("utf-8"),
                    (
                        QUEST_IP,
                        QUEST_PORT,
                    ),
                )

                dashboard_packet = {
                    "schema": DASHBOARD_SCHEMA,
                    "sequence": int(seq),
                    "unix_time_s": sample_unix_time,
                    "monotonic_s": time.monotonic(),
                    "robot": {
                        "model_family": "Unitree G1",
                        "joint_count": NUM_JOINTS,
                        "mode_machine": mode_machine,
                        "mode_pr": mode_pr,
                        "tick": tick,
                        "sample_age_s": sample_age,
                        "dds_callback_count": callback_count,
                        "motor_indices": list(
                            range(NUM_JOINTS)
                        ),
                        "joint_names": list(
                            JOINT_NAMES
                        ),
                        "measured_q_rad": q,
                        "measured_dq_rps": dq,
                        "tau_est": tau_est,
                        "temperatures_c":
                            temperatures,
                        "motor_state": motor_state,
                    },
                }

                try:
                    encoded = json.dumps(
                        dashboard_packet,
                        separators=(",", ":"),
                        allow_nan=False,
                    ).encode("utf-8")

                    sock.sendto(
                        encoded,
                        (
                            DASHBOARD_HOST,
                            DASHBOARD_PORT,
                        ),
                    )
                except Exception as exc:
                    if (
                        seq
                        % max(1, int(SEND_HZ))
                        == 0
                    ):
                        print(
                            "[WARN] Dashboard robot "
                            "telemetry send failed: "
                            f"{exc}"
                        )

                # Print once per second.
                if seq % int(SEND_HZ) == 0:
                    coordinate_name = (
                        "PR"
                        if mode_pr == 0
                        else "AB"
                        if mode_pr == 1
                        else "UNKNOWN"
                    )

                    print(
                        f"seq={seq:8d} "
                        f"tick={tick:12d} "
                        f"mode_pr={mode_pr}({coordinate_name}) "
                        f"machine={mode_machine:2d} "
                        f"L_sh_pitch={q[15]:+.3f} "
                        f"R_sh_pitch={q[22]:+.3f} "
                        f"age={sample_age * 1000.0:.1f}ms"
                    )

                seq += 1

            elapsed = (
                time.monotonic() - loop_start
            )

            sleep_time = period - elapsed

            if sleep_time > 0.0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print()
        print(
            "Stopping full-body telemetry sender..."
        )

    finally:
        try:
            subscriber.Close()
        except Exception:
            pass

        sock.close()

        print("Sender stopped.")


if __name__ == "__main__":
    main()
