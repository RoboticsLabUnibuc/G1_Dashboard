#!/usr/bin/env python3

import csv
import signal
import sys
import time
from pathlib import Path
from threading import Lock

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    LowCmd_,
    LowState_,
)

RUN = True
LOCK = Lock()

latest_state = None
latest_state_t = 0.0
latest_cmd = None
latest_cmd_t = 0.0

JOINTS = {
    15: "L_shoulder_pitch",
    16: "L_shoulder_roll",
    17: "L_shoulder_yaw",
    18: "L_elbow",
    19: "L_wrist_roll",
    20: "L_wrist_pitch",
    21: "L_wrist_yaw",
    22: "R_shoulder_pitch",
    23: "R_shoulder_roll",
    24: "R_shoulder_yaw",
    25: "R_elbow",
    26: "R_wrist_roll",
    27: "R_wrist_pitch",
    28: "R_wrist_yaw",
}

def stop_handler(*_args):
    global RUN
    RUN = False

def lowstate_cb(msg):
    global latest_state, latest_state_t
    with LOCK:
        latest_state = msg
        latest_state_t = time.monotonic()

def armcmd_cb(msg):
    global latest_cmd, latest_cmd_t
    with LOCK:
        latest_cmd = msg
        latest_cmd_t = time.monotonic()

def temperatures(ms):
    try:
        t = list(ms.temperature)
        if len(t) >= 2:
            return int(t[0]), int(t[1])
    except Exception:
        pass
    return -1, -1

def main():
    if len(sys.argv) != 3:
        print(
            "usage: read_only_arm_diagnostic_logger.py "
            "<network-interface> <output.csv>"
        )
        return 2

    iface = sys.argv[1]
    output = Path(sys.argv[2])

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    ChannelFactoryInitialize(0, networkInterface=iface)

    state_sub = ChannelSubscriber("rt/lowstate", LowState_)
    state_sub.Init(lowstate_cb, 1)

    cmd_sub = ChannelSubscriber("rt/arm_sdk", LowCmd_)
    cmd_sub.Init(armcmd_cb, 1)

    fields = [
        "unix_time",
        "monotonic",
        "state_age_s",
        "cmd_age_s",
        "arm_weight",
    ]

    for _, name in JOINTS.items():
        fields += [
            f"{name}_q_meas",
            f"{name}_dq",
            f"{name}_tau_est",
            f"{name}_temp0",
            f"{name}_temp1",
            f"{name}_vol",
            f"{name}_motorstate",
            f"{name}_q_cmd",
            f"{name}_tau_cmd",
        ]

    output.parent.mkdir(parents=True, exist_ok=True)

    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        print(f"READ-ONLY logger writing: {output}")
        print("Ctrl+C stops logger.")
        print("No DDS publisher is created.")

        while RUN:
            now = time.monotonic()

            with LOCK:
                state = latest_state
                state_t = latest_state_t
                cmd = latest_cmd
                cmd_t = latest_cmd_t

            if state is not None:
                row = {
                    "unix_time": f"{time.time():.6f}",
                    "monotonic": f"{now:.6f}",
                    "state_age_s": f"{now-state_t:.6f}",
                    "cmd_age_s": (
                        f"{now-cmd_t:.6f}"
                        if cmd is not None
                        else ""
                    ),
                    "arm_weight": (
                        float(cmd.motor_cmd[29].q)
                        if cmd is not None
                        else ""
                    ),
                }

                for index, name in JOINTS.items():
                    ms = state.motor_state[index]
                    t0, t1 = temperatures(ms)

                    row[f"{name}_q_meas"] = float(ms.q)
                    row[f"{name}_dq"] = float(ms.dq)

                    try:
                        row[f"{name}_tau_est"] = float(ms.tau_est)
                    except Exception:
                        row[f"{name}_tau_est"] = ""

                    row[f"{name}_temp0"] = t0
                    row[f"{name}_temp1"] = t1

                    try:
                        row[f"{name}_vol"] = float(ms.vol)
                    except Exception:
                        row[f"{name}_vol"] = ""

                    try:
                        row[f"{name}_motorstate"] = int(ms.motorstate)
                    except Exception:
                        row[f"{name}_motorstate"] = ""

                    if cmd is not None:
                        row[f"{name}_q_cmd"] = float(
                            cmd.motor_cmd[index].q
                        )
                        row[f"{name}_tau_cmd"] = float(
                            cmd.motor_cmd[index].tau
                        )
                    else:
                        row[f"{name}_q_cmd"] = ""
                        row[f"{name}_tau_cmd"] = ""

                writer.writerow(row)
                f.flush()

            time.sleep(0.02)  # 50 Hz diagnostic logging

    try:
        state_sub.Close()
    except Exception:
        pass

    try:
        cmd_sub.Close()
    except Exception:
        pass

    print("Logger stopped.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
