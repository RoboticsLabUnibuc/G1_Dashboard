#!/usr/bin/env python3
"""Quest-to-PC2 tracking test. Creates no DDS participant or robot publisher."""

from __future__ import annotations

import time
import traceback

import numpy as np

from televuer import TeleVuerWrapper
from teleop_arm_only_direct_headyaw_registered_clutch import (
    raw_head_pose,
    tracking_frame_valid,
)


def xyz(pose: object) -> str:
    try:
        matrix = np.asarray(pose, dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            return "invalid"
        return np.array2string(
            matrix[:3, 3],
            precision=3,
            suppress_small=True,
        )
    except Exception:
        return "invalid"


def main() -> int:
    wrapper: TeleVuerWrapper | None = None

    try:
        print("QUEST TRACKING TEST")
        print("NO DDS participant is created.")
        print("NO robot publisher is created.")
        print(
            "Open: "
            "https://192.168.0.116:8012/"
            "?ws=wss://192.168.0.116:8012"
        )

        wrapper = TeleVuerWrapper(
            use_hand_tracking=True,
            binocular=False,
            img_shape=(480, 640),
            display_mode="pass-through",
            zmq=False,
            webrtc=False,
            webrtc_url=None,
            arm_reference_mode="head_yaw",
        )

        last_report = 0.0
        valid_frames = 0
        invalid_frames = 0

        while True:
            tele_data = wrapper.get_tele_data()
            valid, reason = tracking_frame_valid(wrapper, tele_data)
            head_pose = raw_head_pose(wrapper)

            if valid and head_pose is not None:
                valid_frames += 1
            else:
                invalid_frames += 1

            now = time.monotonic()
            if now - last_report >= 1.0:
                if valid and head_pose is not None:
                    print(
                        "XR=ok"
                        f" | valid={valid_frames}"
                        f" invalid={invalid_frames}"
                        f" | head={xyz(head_pose)}"
                        f" | left={xyz(tele_data.left_wrist_pose)}"
                        f" | right={xyz(tele_data.right_wrist_pose)}",
                        flush=True,
                    )
                else:
                    print(
                        "XR=not-ready"
                        f" | reason={reason}"
                        f" | valid={valid_frames}"
                        f" invalid={invalid_frames}",
                        flush=True,
                    )

                last_report = now

            time.sleep(1.0 / 60.0)

    except KeyboardInterrupt:
        print("\nControlled test exit.")
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        if wrapper is not None:
            try:
                wrapper.close()
            except Exception as exc:
                print(f"TeleVuer close warning: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
