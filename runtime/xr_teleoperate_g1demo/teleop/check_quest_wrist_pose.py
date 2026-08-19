import time
import numpy as np

from televuer import TeleVuerWrapper

np.set_printoptions(precision=3, suppress=True)

tv = TeleVuerWrapper(
    use_hand_tracking=True,
    binocular=False,
    img_shape=(480, 640),
    display_mode="pass-through",
    zmq=False,
    webrtc=False,
    arm_reference_mode="head_yaw",
)

print("Connect the Quest to:")
print("https://vuer.ai?ws=wss://192.168.0.157:8012&grid=False")
print("Enter VR and keep both hands visible.")

while True:
    data = tv.get_tele_data()

    if data.motion_data_ready:
        left = data.left_wrist_pose[:3, 3]
        right = data.right_wrist_pose[:3, 3]

        print(
            f"L xyz: {left}    "
            f"R xyz: {right}",
            flush=True,
        )

    time.sleep(0.20)
