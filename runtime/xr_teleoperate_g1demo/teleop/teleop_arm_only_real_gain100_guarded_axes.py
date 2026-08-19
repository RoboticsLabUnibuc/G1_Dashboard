import time
import argparse
import numpy as np
import pinocchio as pin
from multiprocessing import Value, Array, Lock
import threading
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, H2_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }


# ARM_RELATIVE_CALIBRATION_V1
def get_current_wrist_poses_from_fk(arm_ik, arm_q):
    """Return the current left and right robot end-effector poses."""
    q = np.asarray(arm_q, dtype=float).reshape(-1)

    model = arm_ik.reduced_robot.model
    data = arm_ik.reduced_robot.data

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    def frame_matrix(frame_id):
        placement = data.oMf[frame_id]
        pose = np.eye(4, dtype=float)
        pose[:3, :3] = placement.rotation
        pose[:3, 3] = placement.translation
        return pose

    return (
        frame_matrix(arm_ik.L_hand_id),
        frame_matrix(arm_ik.R_hand_id),
    )



# GUARDED_TRACKING_V1

# TeleVuer adds this waist-frame offset after applying the live head frame.
_TELEVUER_WAIST_OFFSET = np.array(
    [0.15, 0.0, 0.45],
    dtype=float,
)


def get_head_yaw_rotation(head_pose):
    """Extract the headset yaw rotation in the robot basis."""
    x_axis = np.asarray(
        head_pose[:3, 0],
        dtype=float,
    ).copy()

    x_axis[2] = 0.0
    norm = np.linalg.norm(x_axis)

    if not np.isfinite(norm) or norm < 1e-6:
        return np.eye(3)

    x_axis /= norm
    z_axis = np.array([0.0, 0.0, 1.0])
    y_axis = np.cross(z_axis, x_axis)

    y_norm = np.linalg.norm(y_axis)
    if not np.isfinite(y_norm) or y_norm < 1e-6:
        return np.eye(3)

    y_axis /= y_norm

    return np.column_stack(
        [x_axis, y_axis, z_axis]
    )


def yaw_rotation_z(angle_rad):
    """Return a 3-D rotation about the robot vertical axis."""
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)

    return np.array(
        [
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def express_pose_in_frozen_head_yaw(
    pose,
    current_head_yaw,
    frozen_head_yaw,
):
    """
    Convert a pose already expressed in TeleVuer's live head-yaw frame
    into the head-yaw frame captured during arm calibration.
    """
    pose = np.asarray(pose, dtype=float)
    corrected = pose.copy()

    correction = frozen_head_yaw.T @ current_head_yaw

    corrected[:3, 3] = (
        correction
        @ (pose[:3, 3] - _TELEVUER_WAIST_OFFSET)
        + _TELEVUER_WAIST_OFFSET
    )

    corrected[:3, :3] = (
        correction @ pose[:3, :3]
    )

    return corrected


def pose_is_valid(pose):
    """Basic finite SE(3) validity test."""
    pose = np.asarray(pose, dtype=float)

    if pose.shape != (4, 4):
        return False

    if not np.all(np.isfinite(pose)):
        return False

    if not np.allclose(
        pose[3],
        np.array([0.0, 0.0, 0.0, 1.0]),
        atol=1e-3,
    ):
        return False

    rotation = pose[:3, :3]
    determinant = np.linalg.det(rotation)

    if not np.isfinite(determinant):
        return False

    if not 0.8 < determinant < 1.2:
        return False

    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3),
        atol=2e-2,
    ):
        return False

    return True


def rotation_distance_deg(first_pose, second_pose):
    """Angular difference between two pose rotations."""
    relative = (
        first_pose[:3, :3].T
        @ second_pose[:3, :3]
    )

    cosine = (
        np.trace(relative) - 1.0
    ) / 2.0

    cosine = np.clip(cosine, -1.0, 1.0)

    return float(
        np.degrees(np.arccos(cosine))
    )


def pose_jump_within(
    previous_pose,
    candidate_pose,
    max_translation,
    max_rotation_deg,
):
    """Check whether a pose change is plausible for one XR frame."""
    translation_jump = np.linalg.norm(
        candidate_pose[:3, 3]
        - previous_pose[:3, 3]
    )

    rotation_jump = rotation_distance_deg(
        previous_pose,
        candidate_pose,
    )

    return (
        translation_jump <= max_translation
        and rotation_jump <= max_rotation_deg
    )


def hand_tracking_frame_valid(tele_data):
    """
    TeleVuer returns all-zero hand positions when an arm matrix is invalid.
    Treat that condition as a tracking failure.
    """
    if not tele_data.motion_data_ready:
        return False

    if not pose_is_valid(tele_data.head_pose):
        return False

    if not pose_is_valid(tele_data.left_wrist_pose):
        return False

    if not pose_is_valid(tele_data.right_wrist_pose):
        return False

    if tele_data.left_hand_pos is None:
        return False

    if tele_data.right_hand_pos is None:
        return False

    left_hand = np.asarray(
        tele_data.left_hand_pos,
        dtype=float,
    )

    right_hand = np.asarray(
        tele_data.right_hand_pos,
        dtype=float,
    )

    if not np.all(np.isfinite(left_hand)):
        return False

    if not np.all(np.isfinite(right_hand)):
        return False

    # The wrapper sets these arrays to zero when wrist tracking is invalid.
    if np.linalg.norm(left_hand) < 1e-6:
        return False

    if np.linalg.norm(right_hand) < 1e-6:
        return False

    return True


def update_guarded_pose(
    candidate_pose,
    last_good_pose,
    pending_pose,
    pending_count,
    max_jump_m,
    max_jump_deg,
    reacquire_frames,
):
    """
    Accept normal pose changes immediately.

    A large jump must remain stable for several frames before it is accepted.
    Until then, hold the last valid pose.
    """
    candidate_pose = candidate_pose.copy()

    if last_good_pose is None:
        return (
            candidate_pose,
            candidate_pose,
            None,
            0,
            False,
            False,
        )

    if pose_jump_within(
        last_good_pose,
        candidate_pose,
        max_jump_m,
        max_jump_deg,
    ):
        had_pending_jump = pending_count > 0

        return (
            candidate_pose,
            candidate_pose,
            None,
            0,
            False,
            had_pending_jump,
        )

    pending_translation_limit = max(
        max_jump_m * 0.30,
        0.01,
    )

    pending_rotation_limit = max(
        max_jump_deg * 0.30,
        5.0,
    )

    if (
        pending_pose is not None
        and pose_jump_within(
            pending_pose,
            candidate_pose,
            pending_translation_limit,
            pending_rotation_limit,
        )
    ):
        pending_count += 1
    else:
        pending_pose = candidate_pose.copy()
        pending_count = 1

    if pending_count >= reacquire_frames:
        return (
            candidate_pose,
            candidate_pose,
            None,
            0,
            False,
            True,
        )

    return (
        last_good_pose.copy(),
        last_good_pose,
        pending_pose,
        pending_count,
        True,
        False,
    )


def rate_limit_pose(
    previous_pose,
    target_pose,
    dt,
    max_linear_speed,
    max_angular_speed_deg,
):
    """Rate-limit a Cartesian wrist target before IK."""
    if previous_pose is None:
        return target_pose.copy()

    result = previous_pose.copy()

    translation_delta = (
        target_pose[:3, 3]
        - previous_pose[:3, 3]
    )

    translation_distance = np.linalg.norm(
        translation_delta
    )

    max_translation_step = (
        max_linear_speed * dt
    )

    if (
        translation_distance > max_translation_step
        and translation_distance > 1e-9
    ):
        translation_delta *= (
            max_translation_step
            / translation_distance
        )

    result[:3, 3] = (
        previous_pose[:3, 3]
        + translation_delta
    )

    rotation_delta = (
        previous_pose[:3, :3].T
        @ target_pose[:3, :3]
    )

    rotation_vector = np.asarray(
        pin.log3(rotation_delta),
        dtype=float,
    ).reshape(3)

    rotation_angle = np.linalg.norm(
        rotation_vector
    )

    max_rotation_step = (
        np.radians(max_angular_speed_deg)
        * dt
    )

    if (
        rotation_angle > max_rotation_step
        and rotation_angle > 1e-9
    ):
        rotation_vector *= (
            max_rotation_step
            / rotation_angle
        )

    result[:3, :3] = (
        previous_pose[:3, :3]
        @ pin.exp3(rotation_vector)
    )

    return result

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'H2'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    # network parameters
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--no-image-server', action='store_true', help='Use Quest pass-through without Teleimager or robot cameras')
    parser.add_argument(
        '--tracking-jump-m',
        type=float,
        default=0.10,
        help='Maximum accepted single-frame wrist translation jump',
    )
    parser.add_argument(
        '--tracking-jump-deg',
        type=float,
        default=40.0,
        help='Maximum accepted single-frame wrist rotation jump',
    )
    parser.add_argument(
        '--tracking-reacquire-frames',
        type=int,
        default=8,
        help='Stable frames required after an implausible tracking jump',
    )
    parser.add_argument(
        '--max-wrist-speed',
        type=float,
        default=0.35,
        help='Maximum Cartesian wrist-target speed in metres per second',
    )
    parser.add_argument(
        '--max-wrist-rotation-speed-deg',
        type=float,
        default=120.0,
        help='Maximum wrist-target angular speed in degrees per second',
    )
    parser.add_argument(
        '--operator-yaw-offset-deg',
        type=float,
        default=0.0,
        help='Optional horizontal mapping correction in degrees',
    )
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    parser.add_argument(
        '--translation-yaw-offset-deg',
        type=float,
        default=180.0,
        help=(
            'Rotate only Cartesian translation deltas around the '
            'robot vertical axis; wrist orientation is unchanged'
        ),
    )

    args = parser.parse_args()
    logger_mp.debug(f"args: {args}")

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # ARM_ONLY_NO_IMAGE_SERVER_V1
        # For the first real-robot arm-only test we use Quest pass-through.
        # No Teleimager service or physical camera configuration is required.
        if args.no_image_server:
            if args.display_mode != "pass-through":
                raise ValueError(
                    "--no-image-server requires "
                    "--display-mode=pass-through"
                )

            img_client = None
            camera_config = {
                "head_camera": {
                    "binocular": False,
                    "image_shape": (480, 640),
                    "enable_zmq": False,
                    "enable_webrtc": False,
                    "webrtc_port": 60001,
                },
                "left_wrist_camera": {
                    "enable_zmq": False,
                },
                "right_wrist_camera": {
                    "enable_zmq": False,
                },
            }

            xr_need_local_img = False
            logger_mp.info(
                "Image server disabled: using Quest pass-through only."
            )
        else:
            img_client = ImageClient(
                host=args.img_server_ip,
                request_bgr=True,
            )
            camera_config = img_client.get_cam_config()
            logger_mp.debug(f"Camera config: {camera_config}")
            xr_need_local_img = not (
                args.display_mode == "pass-through"
                or camera_config["head_camera"]["enable_webrtc"]
            )

        tv_wrapper = TeleVuerWrapper(
            use_hand_tracking=args.input_mode == "hand",
            binocular=camera_config["head_camera"]["binocular"],
            img_shape=camera_config["head_camera"]["image_shape"],
            display_mode=args.display_mode,
            zmq=camera_config["head_camera"]["enable_zmq"],
            webrtc=camera_config["head_camera"]["enable_webrtc"],
            webrtc_url=(
                None
                if args.no_image_server
                else (
                    f"https://{args.img_server_ip}:"
                    f"{camera_config['head_camera']['webrtc_port']}/offer"
                )
            ),
            arm_reference_mode="head_yaw",
        )

        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "H2":
            arm_ik = H2_ArmIK()
            arm_ctrl = H2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)

        # end-effector
        xr_motion_data_ready = Value('b', False, lock=True)        # [input] whether XR hand/controller motion data has arrived
        if args.ee in ("dex3", "inspire_ftp", "inspire_dfx") and args.input_mode == "controller":
            raise ValueError(f"{args.ee} does not support controller input mode.")
        elif args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "hand":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_hand
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_hand(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                                dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        elif args.ee == "brainco" and args.input_mode == "controller":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller_ctrl
            left_gripper_trigger_in = Value('d', 10.0, lock=True)  # [input]
            left_gripper_squeeze_in = Value('d', 0.0, lock=True)   # [input]
            right_gripper_trigger_in = Value('d', 10.0, lock=True) # [input]
            right_gripper_squeeze_in = Value('d', 0.0, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller_ctrl(left_gripper_trigger_in, left_gripper_squeeze_in, right_gripper_trigger_in, right_gripper_squeeze_in,
                                                dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim, xr_motion_data_ready_in=xr_motion_data_ready)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)           # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] to start syncing the robot with your movements.")
        if args.record:
            logger_mp.info("🟡  Press [s] to START or SAVE recording (toggle cycle).")
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] to stop and exit the program.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
        arm_ctrl.speed_gradual_max()

        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        # Relative XR-to-robot arm calibration state.
        arm_calibrated = False
        calibration_ready_frames = 0
        xr_left_start_pose = None
        xr_right_start_pose = None
        robot_left_start_pose = None
        robot_right_start_pose = None
        arm_position_gain = 1.0


        # GUARDED_TRACKING_V1 state.
        frozen_head_yaw = None

        last_good_left_pose = None
        last_good_right_pose = None

        left_pending_pose = None
        right_pending_pose = None

        left_pending_count = 0
        right_pending_count = 0

        left_guard_active = False
        right_guard_active = False
        tracking_frame_lost = False

        last_left_command = None
        last_right_command = None
        last_command_time = time.time()


        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
            if camera_config['left_wrist_camera']['enable_zmq']:
                if args.record:
                    left_wrist_img = img_client.get_left_wrist_frame()
            if camera_config['right_wrist_camera']['enable_zmq']:
                if args.record:
                    right_wrist_img = img_client.get_right_wrist_frame()

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            # get xr's tele data
            tele_data = tv_wrapper.get_tele_data()
            if args.ee in ("dex3", "inspire_ftp", "inspire_dfx", "brainco")  and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "brainco" and args.input_mode == "controller":
                with left_gripper_trigger_in.get_lock():
                    left_gripper_trigger_in.value = tele_data.left_ctrl_triggerValue
                with left_gripper_squeeze_in.get_lock():
                    left_gripper_squeeze_in.value = tele_data.left_ctrl_squeezeValue
                with right_gripper_trigger_in.get_lock():
                    right_gripper_trigger_in.value = tele_data.right_ctrl_triggerValue
                with right_gripper_squeeze_in.get_lock():
                    right_gripper_squeeze_in.value = tele_data.right_ctrl_squeezeValue
            elif args.ee == "dex1" and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass
            with xr_motion_data_ready.get_lock():
                xr_motion_data_ready.value = tele_data.motion_data_ready
            
            # high level control
            if args.input_mode == "controller" and args.motion:
                # quit teleoperate
                if tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    loco_wrapper.Damp()
                # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                                  -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                                  -tele_data.right_ctrl_thumbstickValue[0]* 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # Solve IK using guarded XR wrist targets.
            time_ik_start = time.time()

            frame_tracking_valid = hand_tracking_frame_valid(
                tele_data
            )

            # Require consecutive valid frames during startup calibration.
            if not arm_calibrated:
                if not frame_tracking_valid:
                    calibration_ready_frames = 0
                    time.sleep(0.01)
                    continue

                calibration_ready_frames += 1

                if calibration_ready_frames == 1:
                    logger_mp.info(
                        "[Arm calibration] Hold both hands and head still..."
                    )

                if calibration_ready_frames < 30:
                    time.sleep(0.01)
                    continue

                current_head_yaw = get_head_yaw_rotation(
                    tele_data.head_pose
                )

                yaw_offset = yaw_rotation_z(
                    np.radians(
                        args.operator_yaw_offset_deg
                    )
                )

                frozen_head_yaw = (
                    current_head_yaw @ yaw_offset
                )

                xr_left_start_pose = express_pose_in_frozen_head_yaw(
                    tele_data.left_wrist_pose,
                    current_head_yaw,
                    frozen_head_yaw,
                )

                xr_right_start_pose = express_pose_in_frozen_head_yaw(
                    tele_data.right_wrist_pose,
                    current_head_yaw,
                    frozen_head_yaw,
                )

                robot_left_start_pose, robot_right_start_pose = (
                    get_current_wrist_poses_from_fk(
                        arm_ik,
                        current_lr_arm_q,
                    )
                )

                last_good_left_pose = (
                    xr_left_start_pose.copy()
                )

                last_good_right_pose = (
                    xr_right_start_pose.copy()
                )

                last_left_command = (
                    robot_left_start_pose.copy()
                )

                last_right_command = (
                    robot_right_start_pose.copy()
                )

                last_command_time = time.time()
                arm_calibrated = True

                logger_mp.info(
                    "[Arm calibration] Complete."
                )

                logger_mp.info(
                    "[Arm calibration] Head-yaw control frame frozen."
                )

                logger_mp.info(
                    "[Arm calibration] Current operator pose is neutral."
                )

                logger_mp.info(
                    "[Tracking guard] "
                    f"jump limits: {args.tracking_jump_m:.2f} m, "
                    f"{args.tracking_jump_deg:.1f} deg; "
                    f"reacquire: {args.tracking_reacquire_frames} frames."
                )

                logger_mp.info(
                    "[Cartesian limiter] "
                    f"{args.max_wrist_speed:.2f} m/s, "
                    f"{args.max_wrist_rotation_speed_deg:.1f} deg/s."
                )

            if frame_tracking_valid:
                if tracking_frame_lost:
                    logger_mp.info(
                        "[Tracking guard] Tracking data returned; "
                        "validating pose continuity."
                    )

                tracking_frame_lost = False

                current_head_yaw = get_head_yaw_rotation(
                    tele_data.head_pose
                )

                left_candidate = express_pose_in_frozen_head_yaw(
                    tele_data.left_wrist_pose,
                    current_head_yaw,
                    frozen_head_yaw,
                )

                right_candidate = express_pose_in_frozen_head_yaw(
                    tele_data.right_wrist_pose,
                    current_head_yaw,
                    frozen_head_yaw,
                )

                (
                    guarded_left_pose,
                    last_good_left_pose,
                    left_pending_pose,
                    left_pending_count,
                    left_rejected,
                    left_reacquired,
                ) = update_guarded_pose(
                    left_candidate,
                    last_good_left_pose,
                    left_pending_pose,
                    left_pending_count,
                    args.tracking_jump_m,
                    args.tracking_jump_deg,
                    args.tracking_reacquire_frames,
                )

                (
                    guarded_right_pose,
                    last_good_right_pose,
                    right_pending_pose,
                    right_pending_count,
                    right_rejected,
                    right_reacquired,
                ) = update_guarded_pose(
                    right_candidate,
                    last_good_right_pose,
                    right_pending_pose,
                    right_pending_count,
                    args.tracking_jump_m,
                    args.tracking_jump_deg,
                    args.tracking_reacquire_frames,
                )

                if left_rejected and not left_guard_active:
                    logger_mp.warning(
                        "[Tracking guard] Left wrist jump rejected; "
                        "holding last valid pose."
                    )

                if right_rejected and not right_guard_active:
                    logger_mp.warning(
                        "[Tracking guard] Right wrist jump rejected; "
                        "holding last valid pose."
                    )

                if left_reacquired:
                    logger_mp.info(
                        "[Tracking guard] Left wrist tracking reacquired."
                    )

                if right_reacquired:
                    logger_mp.info(
                        "[Tracking guard] Right wrist tracking reacquired."
                    )

                left_guard_active = left_rejected
                right_guard_active = right_rejected

            else:
                if not tracking_frame_lost:
                    logger_mp.warning(
                        "[Tracking guard] Hand tracking lost; "
                        "holding both arm targets."
                    )

                tracking_frame_lost = True

                left_pending_pose = None
                right_pending_pose = None
                left_pending_count = 0
                right_pending_count = 0

                guarded_left_pose = (
                    last_good_left_pose.copy()
                )

                guarded_right_pose = (
                    last_good_right_pose.copy()
                )

            desired_left_target = guarded_left_pose.copy()
            desired_right_target = guarded_right_pose.copy()

            # TRANSLATION_FRAME_CORRECTION_V1
            # Rotate translation deltas independently of wrist orientation.
            # A 180-degree offset flips robot X and Y while preserving Z.
            translation_frame_rotation = yaw_rotation_z(
                np.radians(args.translation_yaw_offset_deg)
            )

            left_translation_delta = (
                translation_frame_rotation
                @ (
                    guarded_left_pose[:3, 3]
                    - xr_left_start_pose[:3, 3]
                )
            )

            right_translation_delta = (
                translation_frame_rotation
                @ (
                    guarded_right_pose[:3, 3]
                    - xr_right_start_pose[:3, 3]
                )
            )

            desired_left_target[:3, 3] = (
                robot_left_start_pose[:3, 3]
                + arm_position_gain * left_translation_delta
            )

            desired_right_target[:3, 3] = (
                robot_right_start_pose[:3, 3]
                + arm_position_gain * right_translation_delta
            )

            # Orientation remains relative to the calibration orientation.
            desired_left_target[:3, :3] = (
                robot_left_start_pose[:3, :3]
                @ xr_left_start_pose[:3, :3].T
                @ guarded_left_pose[:3, :3]
            )

            desired_right_target[:3, :3] = (
                robot_right_start_pose[:3, :3]
                @ xr_right_start_pose[:3, :3].T
                @ guarded_right_pose[:3, :3]
            )

            command_time = time.time()

            command_dt = float(
                np.clip(
                    command_time - last_command_time,
                    0.005,
                    0.10,
                )
            )

            last_command_time = command_time

            left_wrist_target = rate_limit_pose(
                last_left_command,
                desired_left_target,
                command_dt,
                args.max_wrist_speed,
                args.max_wrist_rotation_speed_deg,
            )

            right_wrist_target = rate_limit_pose(
                last_right_command,
                desired_right_target,
                command_dt,
                args.max_wrist_speed,
                args.max_wrist_rotation_speed_deg,
            )

            last_left_command = left_wrist_target.copy()
            last_right_command = right_wrist_target.copy()

            sol_q, sol_tauff = arm_ik.solve_ik(
                left_wrist_target,
                right_wrist_target,
                current_lr_arm_q,
                current_lr_arm_dq,
            )

            time_ik_end = time.time()

            logger_mp.debug(
                f"ik:\t{round(time_ik_end - time_ik_start, 6)}"
            )

            arm_ctrl.ctrl_dual_arm(
                sol_q,
                sol_tauff,
            )

            # record data
            if args.record:
                READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                # dex hand or gripper
                if args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                elif (args.ee == "brainco" and args.input_mode == "controller"):
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if camera_config['head_camera']['binocular']:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{2}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{3}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{1}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{2}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
        try:
            arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if not args.motion:
                pass
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to exit debug mode: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        exit(0)
