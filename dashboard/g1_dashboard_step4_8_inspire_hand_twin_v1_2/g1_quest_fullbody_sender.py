#!/usr/bin/env python3

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

if not (1 <= QUEST_PORT <= 65535):
    raise SystemExit("invalid Quest telemetry UDP port")
if not (1.0 <= SEND_HZ <= 120.0):
    raise SystemExit("invalid Quest telemetry send rate")


# ============================================================
# SHARED LOWSTATE DATA
# ============================================================

lock = threading.Lock()

latest_q = None
latest_tick = -1

# mode_pr:
#   0 = PR mode (Pitch/Roll coordinates)
#   1 = AB mode (parallel A/B coordinates)
latest_mode_pr = -1

# G1 hardware/model configuration identifier.
latest_mode_machine = -1

latest_recv = 0.0

stop_event = threading.Event()


def request_stop(_signum, _frame):
    stop_event.set()


# ============================================================
# DDS CALLBACK
# ============================================================

def lowstate_callback(msg):
    global latest_q
    global latest_tick
    global latest_mode_pr
    global latest_mode_machine
    global latest_recv

    try:
        q = [
            float(msg.motor_state[i].q)
            for i in range(NUM_JOINTS)
        ]

        tick = int(msg.tick)

        mode_pr = int(msg.mode_pr)
        mode_machine = int(msg.mode_machine)

    except Exception as exc:
        print(
            f"[WARN] Failed to parse LowState: {exc}"
        )
        return

    with lock:
        latest_q = q
        latest_tick = tick
        latest_mode_pr = mode_pr
        latest_mode_machine = mode_machine
        latest_recv = time.monotonic()


# ============================================================
# MAIN
# ============================================================

def main():
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    print("==========================================")
    print(" G1 -> QUEST FULL-BODY TELEMETRY")
    print("==========================================")
    print()

    print(
        f"Quest destination : "
        f"{QUEST_IP}:{QUEST_PORT}"
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

    # queueLen = 0 -> latest-data style callback
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
                if latest_q is None:
                    q = None
                else:
                    q = latest_q.copy()

                tick = latest_tick
                mode_pr = latest_mode_pr
                mode_machine = latest_mode_machine
                recv = latest_recv

            if q is not None:
                sample_age = max(
                    0.0,
                    time.monotonic() - recv,
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
