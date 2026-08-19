#!/usr/bin/env python3
"""Quest hand-tracking -> Inspire DFX finger-only test V3 for Unitree G1 PC2.

This program publishes ONLY to ``rt/inspire/cmd``.
It does not construct an arm controller and does not publish ``rt/arm_sdk``.

Keys
----
r : arm/disarm Quest finger following
o : disarm and command a slow fully-open pose
p : print a state/action snapshot
q : slow open and exit

Safety behavior
---------------
- Starts disarmed and commands both hands fully open.
- Requires stable, finite 25x3 landmarks for both hands before arming.
- Rate-limits every Inspire actuator command.
- Limits first-test closure with ``--minimum-command``.
  Inspire DFX convention: 1.0=open, 0.0=closed.
- Tracking loss disarms control and smoothly opens both hands.
- Normal exit and Ctrl+C smoothly open both hands before stopping.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
from sshkeyboard import listen_keyboard, stop_listening

CURRENT_DIR = Path(__file__).resolve().parent
REPO_ROOT = CURRENT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# hand_retargeting.py uses paths relative to teleop/.
os.chdir(CURRENT_DIR)

from televuer import TeleVuerWrapper
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_


COMMAND_TOPIC = "rt/inspire/cmd"
STATE_TOPIC = "rt/inspire/state"
MOTOR_COUNT = 12

# DFX message order:
# right pinky/ring/middle/index/thumb-bend/thumb-rotation, then left.
RIGHT_IDS = tuple(range(0, 6))
LEFT_IDS = tuple(range(6, 12))

ARMED_TOGGLE = False
OPEN_REQUESTED = False
SNAPSHOT_REQUESTED = False
QUIT_REQUESTED = False
STOP_EVENT = threading.Event()
SIGNAL_COUNT = 0
SHUTTING_DOWN = False


def build_logger() -> logging.Logger:
    logger = logging.getLogger("quest_inspire_dfx_finger_only")
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)-8s [FingerOnly] %(message)s",
            "%H:%M:%S",
        )
    )
    logger.addHandler(handler)
    logger.propagate = False
    return logger


LOG = build_logger()


def on_press(key: str) -> None:
    global ARMED_TOGGLE
    global OPEN_REQUESTED
    global SNAPSHOT_REQUESTED
    global QUIT_REQUESTED

    if key == "r":
        ARMED_TOGGLE = True
    elif key == "o":
        OPEN_REQUESTED = True
    elif key == "p":
        SNAPSHOT_REQUESTED = True
    elif key == "q":
        QUIT_REQUESTED = True


def signal_handler(signum: int, _frame: object) -> None:
    global SIGNAL_COUNT
    global QUIT_REQUESTED

    # sshkeyboard/TeleVuer cleanup may emit SIGTERM internally. Once the
    # hands are already opening and cleanup has begun, ignore those signals.
    if SHUTTING_DOWN:
        return

    SIGNAL_COUNT += 1
    if SIGNAL_COUNT == 1:
        LOG.warning(
            "Signal %s received. Requesting controlled hand opening.",
            signum,
        )
        QUIT_REQUESTED = True
    else:
        LOG.error("Second external signal received. Forcing loop exit.")
        STOP_EVENT.set()


def finite_hand_landmarks(value: object) -> tuple[bool, Optional[np.ndarray], str]:
    try:
        data = np.asarray(value, dtype=float)
    except Exception as exc:
        return False, None, f"conversion failed: {exc}"

    if data.shape != (25, 3):
        return False, None, f"shape={data.shape}, expected (25, 3)"

    if not np.isfinite(data).all():
        return False, None, "contains NaN/Inf"

    # A hidden/uninitialized hand is commonly an all-zero or collapsed cloud.
    extents = np.ptp(data, axis=0)
    max_extent = float(np.max(extents))
    if max_extent < 0.025:
        return False, None, f"collapsed landmarks, extent={max_extent:.4f} m"

    return True, data, "ok"


def normalize_inspire_target(raw_target: np.ndarray) -> np.ndarray:
    raw = np.asarray(raw_target, dtype=float).copy()
    if raw.shape != (6,):
        raise ValueError(f"Expected six Inspire targets, got {raw.shape}")

    result = np.empty(6, dtype=float)

    # Official DFX convention after normalization:
    # 1.0 fully open, 0.0 fully closed.
    result[:4] = (1.7 - raw[:4]) / 1.7
    result[4] = (0.5 - raw[4]) / 0.5
    result[5] = (1.3 - raw[5]) / 1.4

    return np.clip(result, 0.0, 1.0)


def rate_limit(
    current: np.ndarray,
    target: np.ndarray,
    max_speed_per_s: float,
    dt: float,
) -> np.ndarray:
    step = max(0.0, float(max_speed_per_s)) * max(0.0, float(dt))
    return current + np.clip(target - current, -step, step)


class InspireDfxIo:
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

        self.last_state: Optional[np.ndarray] = None
        self.last_state_time = 0.0

    def publish(self, left: np.ndarray, right: np.ndarray) -> None:
        left = np.asarray(left, dtype=float)
        right = np.asarray(right, dtype=float)
        if left.shape != (6,) or right.shape != (6,):
            raise ValueError("Left/right commands must each contain six values")

        right = np.clip(right, 0.0, 1.0)
        left = np.clip(left, 0.0, 1.0)

        for index, motor_id in enumerate(RIGHT_IDS):
            self.message.cmds[motor_id].q = float(right[index])
        for index, motor_id in enumerate(LEFT_IDS):
            self.message.cmds[motor_id].q = float(left[index])

        self.publisher.Write(self.message)

    def poll_state(self) -> Optional[np.ndarray]:
        message = self.subscriber.Read()
        if message is None:
            return self.last_state

        try:
            values = np.asarray(
                [message.states[index].q for index in range(MOTOR_COUNT)],
                dtype=float,
            )
        except Exception as exc:
            LOG.warning("Could not decode Inspire state: %s", exc)
            return self.last_state

        if np.isfinite(values).all():
            self.last_state = values
            self.last_state_time = time.monotonic()

        return self.last_state


def wait_for_state(io: InspireDfxIo, timeout_s: float) -> np.ndarray:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = io.poll_state()
        if state is not None:
            LOG.info("Inspire state feedback received.")
            return state
        time.sleep(0.01)
    raise TimeoutError(
        f"No {STATE_TOPIC} feedback within {timeout_s:.1f} seconds. "
        "Confirm inspire_g1 is running with /usr/local DDS."
    )


def main() -> int:
    global SHUTTING_DOWN
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--network-interface",
        required=True,
        help="DDS interface, expected enP8p1s0 on G1 PC2.",
    )
    parser.add_argument("--frequency", type=float, default=30.0)
    parser.add_argument(
        "--command-speed-per-s",
        type=float,
        default=0.40,
        help="Maximum normalized actuator change per second.",
    )
    parser.add_argument(
        "--minimum-command",
        type=float,
        default=0.20,
        help=(
            "Lowest allowed command during this test. "
            "1=open, 0=fully closed; default prevents hard closure."
        ),
    )
    parser.add_argument(
        "--stable-tracking-frames",
        type=int,
        default=15,
    )
    parser.add_argument(
        "--tracking-fault-frames",
        type=int,
        default=3,
    )
    parser.add_argument(
        "--state-timeout-s",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--status-hz",
        type=float,
        default=2.0,
    )
    args = parser.parse_args()

    if args.frequency <= 0:
        raise ValueError("--frequency must be positive")
    if args.command_speed_per_s <= 0:
        raise ValueError("--command-speed-per-s must be positive")
    if not 0.0 <= args.minimum_command < 1.0:
        raise ValueError("--minimum-command must be in [0, 1)")
    if args.stable_tracking_frames < 1:
        raise ValueError("--stable-tracking-frames must be >=1")
    if args.tracking_fault_frames < 1:
        raise ValueError("--tracking-fault-frames must be >=1")
    if args.status_hz <= 0:
        raise ValueError("--status-hz must be positive")

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    keyboard_thread = threading.Thread(
        target=listen_keyboard,
        kwargs={
            "on_press": on_press,
            "until": None,
            "sequential": False,
        },
        daemon=True,
    )
    keyboard_thread.start()

    tv_wrapper: Optional[TeleVuerWrapper] = None
    io: Optional[InspireDfxIo] = None
    current_left = np.ones(6, dtype=float)
    current_right = np.ones(6, dtype=float)

    try:
        ChannelFactoryInitialize(
            0,
            networkInterface=args.network_interface,
        )

        LOG.warning(
            "FINGER-ONLY LIVE TEST: publishes %s; no arm publisher is created.",
            COMMAND_TOPIC,
        )
        LOG.info(
            "Interface=%s | command speed<=%.2f/s | minimum command=%.2f",
            args.network_interface,
            args.command_speed_per_s,
            args.minimum_command,
        )
        LOG.info("Keys: [r] arm/disarm, [o] open, [p] snapshot, [q] open+exit")
        LOG.info(
            "Open Quest page: "
            "https://<PC2-WIFI-IP>:8012/?ws=wss://<PC2-WIFI-IP>:8012"
        )

        io = InspireDfxIo()
        wait_for_state(io, args.state_timeout_s)

        # Establish a deterministic open pose before starting XR.
        open_deadline = time.monotonic() + 1.5
        last_publish = time.monotonic()
        while time.monotonic() < open_deadline:
            now = time.monotonic()
            dt = max(0.0, now - last_publish)
            last_publish = now
            current_left = rate_limit(
                current_left,
                np.ones(6),
                args.command_speed_per_s,
                dt,
            )
            current_right = rate_limit(
                current_right,
                np.ones(6),
                args.command_speed_per_s,
                dt,
            )
            io.publish(current_left, current_right)
            io.poll_state()
            time.sleep(1.0 / args.frequency)

        tv_wrapper = TeleVuerWrapper(
            use_hand_tracking=True,
            binocular=False,
            img_shape=(480, 640),
            display_mode="pass-through",
            zmq=False,
            webrtc=False,
            webrtc_url=None,
            arm_reference_mode="head_yaw",
        )

        retargeting = HandRetargeting(HandType.INSPIRE_HAND)

        armed = False
        stable_count = 0
        fault_count = 0
        last_reason = "waiting for Quest"
        last_status = 0.0
        last_loop = time.monotonic()
        target_left = np.ones(6, dtype=float)
        target_right = np.ones(6, dtype=float)
        last_raw_left = np.full(6, np.nan, dtype=float)
        last_raw_right = np.full(6, np.nan, dtype=float)
        last_normalized_left = np.ones(6, dtype=float)
        last_normalized_right = np.ones(6, dtype=float)

        while not STOP_EVENT.is_set():
            global ARMED_TOGGLE
            global OPEN_REQUESTED
            global SNAPSHOT_REQUESTED
            global QUIT_REQUESTED

            loop_start = time.monotonic()
            dt = min(0.1, max(0.0, loop_start - last_loop))
            last_loop = loop_start

            tele_data = tv_wrapper.get_tele_data()

            left_valid, left_hand, left_reason = finite_hand_landmarks(
                getattr(tele_data, "left_hand_pos", None)
            )
            right_valid, right_hand, right_reason = finite_hand_landmarks(
                getattr(tele_data, "right_hand_pos", None)
            )
            motion_ready = bool(
                getattr(tele_data, "motion_data_ready", False)
            )
            tracking_valid = (
                motion_ready
                and left_valid
                and right_valid
                and left_hand is not None
                and right_hand is not None
            )

            if tracking_valid:
                stable_count += 1
                fault_count = 0
                last_reason = "ok"
            else:
                stable_count = 0
                fault_count += 1
                last_reason = (
                    f"motion_ready={motion_ready}; "
                    f"left={left_reason}; right={right_reason}"
                )

            if ARMED_TOGGLE:
                ARMED_TOGGLE = False
                if armed:
                    armed = False
                    LOG.warning("DISARMED. Smoothly opening both hands.")
                elif stable_count >= args.stable_tracking_frames:
                    armed = True
                    fault_count = 0
                    LOG.warning(
                        "ARMED. Quest finger tracking now controls Inspire DFX."
                    )
                else:
                    LOG.warning(
                        "Cannot arm: need %d stable frames; currently %d. %s",
                        args.stable_tracking_frames,
                        stable_count,
                        last_reason,
                    )

            if OPEN_REQUESTED:
                OPEN_REQUESTED = False
                armed = False
                LOG.warning("OPEN requested. Finger following disabled.")

            if QUIT_REQUESTED:
                QUIT_REQUESTED = False
                armed = False
                LOG.warning("Exit requested. Opening both hands first.")
                break

            if armed and fault_count >= args.tracking_fault_frames:
                armed = False
                LOG.error(
                    "TRACKING FAULT. Finger following disabled; "
                    "opening both hands. %s",
                    last_reason,
                )

            if armed and tracking_valid:
                assert left_hand is not None
                assert right_hand is not None

                ref_left = (
                    left_hand[retargeting.left_indices[1, :]]
                    - left_hand[retargeting.left_indices[0, :]]
                )
                ref_right = (
                    right_hand[retargeting.right_indices[1, :]]
                    - right_hand[retargeting.right_indices[0, :]]
                )

                raw_left = retargeting.left_retargeting.retarget(ref_left)[
                    retargeting.left_dex_retargeting_to_hardware
                ]
                raw_right = retargeting.right_retargeting.retarget(ref_right)[
                    retargeting.right_dex_retargeting_to_hardware
                ]

                last_raw_left = np.asarray(raw_left, dtype=float).copy()
                last_raw_right = np.asarray(raw_right, dtype=float).copy()

                target_left = normalize_inspire_target(raw_left)
                target_right = normalize_inspire_target(raw_right)
                last_normalized_left = target_left.copy()
                last_normalized_right = target_right.copy()

                # First live test: do not permit hard closure.
                target_left = np.maximum(
                    target_left,
                    args.minimum_command,
                )
                target_right = np.maximum(
                    target_right,
                    args.minimum_command,
                )
            else:
                target_left = np.ones(6, dtype=float)
                target_right = np.ones(6, dtype=float)

            current_left = rate_limit(
                current_left,
                target_left,
                args.command_speed_per_s,
                dt,
            )
            current_right = rate_limit(
                current_right,
                target_right,
                args.command_speed_per_s,
                dt,
            )

            io.publish(current_left, current_right)
            state = io.poll_state()

            if SNAPSHOT_REQUESTED:
                SNAPSHOT_REQUESTED = False
                LOG.info(
                    "SNAPSHOT armed=%s stable=%d faults=%d tracking=%s | "
                    "raw-retarget L=%s R=%s | "
                    "normalized-before-floor L=%s R=%s | "
                    "cmd-after-floor+limiter L=%s R=%s | raw state=%s | "
                    "thumb-rotation(index5) raw L/R=%+.4f/%+.4f "
                    "normalized L/R=%.3f/%.3f cmd L/R=%.3f/%.3f",
                    armed,
                    stable_count,
                    fault_count,
                    last_reason,
                    np.array2string(last_raw_left, precision=4),
                    np.array2string(last_raw_right, precision=4),
                    np.array2string(last_normalized_left, precision=3),
                    np.array2string(last_normalized_right, precision=3),
                    np.array2string(current_left, precision=3),
                    np.array2string(current_right, precision=3),
                    (
                        "unavailable"
                        if state is None
                        else np.array2string(state, precision=3)
                    ),
                    float(last_raw_left[5]),
                    float(last_raw_right[5]),
                    float(last_normalized_left[5]),
                    float(last_normalized_right[5]),
                    float(current_left[5]),
                    float(current_right[5]),
                )

            if loop_start - last_status >= 1.0 / args.status_hz:
                last_status = loop_start
                LOG.info(
                    "armed=%s | tracking=%s | stable=%d | "
                    "Lcmd=%s | Rcmd=%s",
                    armed,
                    last_reason,
                    stable_count,
                    np.array2string(current_left, precision=2),
                    np.array2string(current_right, precision=2),
                )

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, 1.0 / args.frequency - elapsed))

        # Controlled opening before normal exit.
        SHUTTING_DOWN = True
        LOG.warning("Controlled opening in progress.")
        open_deadline = time.monotonic() + 3.5
        last_publish = time.monotonic()

        while time.monotonic() < open_deadline and not STOP_EVENT.is_set():
            now = time.monotonic()
            dt = min(0.1, max(0.0, now - last_publish))
            last_publish = now

            current_left = rate_limit(
                current_left,
                np.ones(6),
                args.command_speed_per_s,
                dt,
            )
            current_right = rate_limit(
                current_right,
                np.ones(6),
                args.command_speed_per_s,
                dt,
            )

            io.publish(current_left, current_right)
            io.poll_state()

            if (
                float(np.min(current_left)) >= 0.995
                and float(np.min(current_right)) >= 0.995
            ):
                break

            time.sleep(1.0 / args.frequency)

        # Publish several identical open samples to ensure service receipt.
        for _ in range(10):
            io.publish(np.ones(6), np.ones(6))
            time.sleep(0.02)

        LOG.info("Both hand commands are fully open. Exiting.")
        return 0

    finally:
        SHUTTING_DOWN = True

        # Ignore external cleanup signals in the parent after both hands have
        # already been commanded open.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        except Exception:
            pass

        # TeleVuer owns a multiprocessing child. Its normal close() uses
        # Process.terminate() (SIGTERM), but the forked child inherited this
        # script's SIGTERM safety handler and therefore refuses to terminate.
        # Kill only that known TeleVuer child after the safe-open sequence.
        if tv_wrapper is not None:
            process = getattr(
                getattr(tv_wrapper, "tvuer", None),
                "process",
                None,
            )
            if process is not None:
                try:
                    if process.is_alive():
                        LOG.info(
                            "Stopping TeleVuer child process pid=%s.",
                            process.pid,
                        )
                        process.kill()
                    process.join(timeout=1.0)
                    if process.is_alive():
                        LOG.warning(
                            "TeleVuer child pid=%s is still alive.",
                            process.pid,
                        )
                    else:
                        LOG.info("TeleVuer child stopped.")
                except Exception as exc:
                    LOG.warning(
                        "Direct TeleVuer child cleanup failed: %s",
                        exc,
                    )

        # The keyboard listener thread is daemonized, but ask it to stop
        # cleanly when supported.
        try:
            stop_listening()
        except Exception as exc:
            LOG.debug("Keyboard cleanup failed: %s", exc)


if __name__ == "__main__":
    exit_code = main()

    # Some TeleVuer/keyboard runtime versions leave workers alive after all
    # hand commands are safely open. Flush logs and terminate this standalone
    # test process deterministically.
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        os._exit(int(exit_code))
