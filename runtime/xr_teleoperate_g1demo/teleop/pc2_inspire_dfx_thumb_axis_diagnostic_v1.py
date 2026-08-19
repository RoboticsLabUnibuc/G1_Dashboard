#!/usr/bin/env python3
"""Direct DDS diagnostic for Inspire DFX thumb-rotation axes.

This bypasses Quest, TeleVuer, and dex-retargeting entirely.

Official DFX command order:
  0..5   right: pinky, ring, middle, index, thumb-bend, thumb-rotation
  6..11  left:  pinky, ring, middle, index, thumb-bend, thumb-rotation

Commands:
  l  sweep only left thumb rotation (DDS motor 11)
  r  sweep only right thumb rotation (DDS motor 5)
  q  command all axes fully open and exit

Inspire normalized convention:
  1.0 = open / one rotation endpoint
  0.0 = closed / opposite endpoint
"""

from __future__ import annotations

import argparse
import time
from typing import Optional

import numpy as np

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_


COMMAND_TOPIC = "rt/inspire/cmd"
STATE_TOPIC = "rt/inspire/state"
RIGHT_THUMB_ROTATION_ID = 5
LEFT_THUMB_ROTATION_ID = 11
MOTOR_COUNT = 12


class DfxIo:
    def __init__(self) -> None:
        self.publisher = ChannelPublisher(COMMAND_TOPIC, MotorCmds_)
        self.publisher.Init()

        self.subscriber = ChannelSubscriber(STATE_TOPIC, MotorStates_)
        self.subscriber.Init()

        self.message = MotorCmds_()
        self.message.cmds = [
            unitree_go_msg_dds__MotorCmd_()
            for _ in range(MOTOR_COUNT)
        ]

    def publish(self, command: np.ndarray) -> None:
        command = np.asarray(command, dtype=float)
        if command.shape != (MOTOR_COUNT,):
            raise ValueError(f"Expected 12 commands, got {command.shape}")
        command = np.clip(command, 0.0, 1.0)

        for index, value in enumerate(command):
            self.message.cmds[index].q = float(value)

        self.publisher.Write(self.message)

    def state(self) -> Optional[np.ndarray]:
        message = self.subscriber.Read()
        if message is None:
            return None
        try:
            return np.asarray(
                [message.states[i].q for i in range(MOTOR_COUNT)],
                dtype=float,
            )
        except Exception:
            return None


def move_axis(
    io: DfxIo,
    command: np.ndarray,
    motor_id: int,
    target: float,
    speed_per_s: float,
    frequency: float,
) -> np.ndarray:
    current = float(command[motor_id])
    target = float(np.clip(target, 0.0, 1.0))
    period = 1.0 / frequency
    max_step = speed_per_s / frequency

    while abs(target - current) > 1e-6:
        delta = float(np.clip(target - current, -max_step, max_step))
        current += delta
        if abs(target - current) < max_step:
            current = target

        command[motor_id] = current
        io.publish(command)
        time.sleep(period)

    return command


def sweep(
    io: DfxIo,
    motor_id: int,
    label: str,
    speed_per_s: float,
    minimum_command: float,
    frequency: float,
) -> None:
    command = np.ones(MOTOR_COUNT, dtype=float)

    print()
    print(f"Testing {label}, DDS motor {motor_id}.")
    print("All other eleven axes remain commanded at 1.0 (open).")
    print(
        f"The tested axis will move slowly 1.0 -> {minimum_command:.2f} -> 1.0."
    )
    input("Keep both hands unobstructed, then press Enter to start...")

    # Reassert all-open before the isolated sweep.
    for _ in range(int(frequency * 0.5)):
        io.publish(command)
        time.sleep(1.0 / frequency)

    state_before = io.state()
    if state_before is not None:
        print(
            "State before: "
            f"right thumb rotation[5]={state_before[5]:.4f}, "
            f"left thumb rotation[11]={state_before[11]:.4f}"
        )

    command = move_axis(
        io,
        command,
        motor_id,
        minimum_command,
        speed_per_s,
        frequency,
    )
    time.sleep(0.8)

    state_low = io.state()
    if state_low is not None:
        print(
            "State at low command: "
            f"right[5]={state_low[5]:.4f}, left[11]={state_low[11]:.4f}"
        )

    command = move_axis(
        io,
        command,
        motor_id,
        1.0,
        speed_per_s,
        frequency,
    )

    for _ in range(int(frequency * 0.5)):
        io.publish(np.ones(MOTOR_COUNT, dtype=float))
        time.sleep(1.0 / frequency)

    state_after = io.state()
    if state_after is not None:
        print(
            "State after reopening: "
            f"right[5]={state_after[5]:.4f}, left[11]={state_after[11]:.4f}"
        )

    print(f"{label} sweep complete; all axes are open.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--network-interface", required=True)
    parser.add_argument("--frequency", type=float, default=60.0)
    parser.add_argument("--speed-per-s", type=float, default=0.60)
    parser.add_argument("--minimum-command", type=float, default=0.20)
    args = parser.parse_args()

    if args.frequency <= 0:
        raise ValueError("--frequency must be positive")
    if args.speed_per_s <= 0:
        raise ValueError("--speed-per-s must be positive")
    if not 0.0 <= args.minimum_command < 1.0:
        raise ValueError("--minimum-command must be in [0, 1)")

    ChannelFactoryInitialize(
        0,
        networkInterface=args.network_interface,
    )
    io = DfxIo()

    open_command = np.ones(MOTOR_COUNT, dtype=float)
    print("DIRECT INSPIRE DFX THUMB-AXIS TEST")
    print("Publishes only rt/inspire/cmd; no Quest or arm controller.")
    print("Waiting for rt/inspire/state...")

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        io.publish(open_command)
        if io.state() is not None:
            print("Inspire state feedback received.")
            break
        time.sleep(0.05)
    else:
        raise TimeoutError(
            "No Inspire state feedback. Confirm inspire_g1 is running."
        )

    try:
        while True:
            print()
            choice = input(
                "[l] left thumb rotation  [r] right thumb rotation  "
                "[q] open and quit: "
            ).strip().lower()

            if choice == "l":
                sweep(
                    io,
                    LEFT_THUMB_ROTATION_ID,
                    "LEFT thumb rotation",
                    args.speed_per_s,
                    args.minimum_command,
                    args.frequency,
                )
            elif choice == "r":
                sweep(
                    io,
                    RIGHT_THUMB_ROTATION_ID,
                    "RIGHT thumb rotation",
                    args.speed_per_s,
                    args.minimum_command,
                    args.frequency,
                )
            elif choice == "q":
                break
            else:
                print("Enter l, r, or q.")
    finally:
        print("Commanding every Inspire axis fully open.")
        for _ in range(int(args.frequency * 1.0)):
            io.publish(open_command)
            time.sleep(1.0 / args.frequency)

    print("All-open command sent. Exiting.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
