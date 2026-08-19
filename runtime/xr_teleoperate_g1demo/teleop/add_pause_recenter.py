#!/usr/bin/env python3
from pathlib import Path
import shutil

SOURCE = Path("teleop_arm_only_real_gain100_autoaxes_saved.py")
TARGET = Path("teleop_arm_only_real_gain100_autoaxes_saved_recenter.py")
BACKUP = Path(
    "teleop_arm_only_real_gain100_autoaxes_saved.WORKING_BACKUP.py"
)
MARKER = "PAUSE_RECENTER_V1"


def fail(message: str) -> None:
    raise SystemExit(f"ERROR: {message}")


if not SOURCE.exists():
    fail(f"Missing source file: {SOURCE.resolve()}")

source_text = SOURCE.read_text()

required_markers = (
    "AUTO_TRANSLATION_REGISTRATION_V1",
    "SAVED_TRANSLATION_PROFILE_V1",
)

for required_marker in required_markers:
    if required_marker not in source_text:
        fail(
            f"{SOURCE} does not contain {required_marker}. "
            "Use the working saved-map teleoperation file."
        )

if MARKER in source_text:
    fail(f"{SOURCE} already contains {MARKER}.")

if not BACKUP.exists():
    shutil.copy2(SOURCE, BACKUP)

shutil.copy2(SOURCE, TARGET)
text = TARGET.read_text()

# ---------------------------------------------------------------------
# 1. Replace the keyboard handler with a version that supports c.
# ---------------------------------------------------------------------
handler_start = text.find("# AUTO_TRANSLATION_REGISTRATION_V1\n")
handler_end = text.find("def get_state() -> dict:\n", handler_start)

if handler_start == -1 or handler_end == -1:
    fail("Could not locate the keyboard-handler section.")

new_handler = '''# AUTO_TRANSLATION_REGISTRATION_V1
# PAUSE_RECENTER_V1
CAPTURE_FORWARD = False
CAPTURE_OUTWARD = False
ENABLE_REGISTERED_CONTROL = False
RECENTER_TOGGLE = False


def on_press(key):
    global STOP, START, RECORD_TOGGLE
    global CAPTURE_FORWARD, CAPTURE_OUTWARD
    global ENABLE_REGISTERED_CONTROL
    global RECENTER_TOGGLE

    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    elif key == 'f' and START == True:
        CAPTURE_FORWARD = True
    elif key == 'o' and START == True:
        CAPTURE_OUTWARD = True
    elif key == 'g' and START == True:
        ENABLE_REGISTERED_CONTROL = True
    elif key == 'c' and START == True:
        RECENTER_TOGGLE = True
    else:
        logger_mp.warning(
            f"[on_press] {key} was pressed, "
            "but no action is defined for this key."
        )


'''

text = (
    text[:handler_start]
    + new_handler
    + text[handler_end:]
)

# ---------------------------------------------------------------------
# 2. Print recenter instructions before the initial r prompt.
# ---------------------------------------------------------------------
startup_anchor = (
    '        logger_mp.info("----------------------------------------------------------------")\n'
)

if text.count(startup_anchor) != 1:
    fail("Could not uniquely locate the startup separator.")

startup_message = '''        logger_mp.info(
            "[Recenter] Press [c] once to PAUSE and hold the robot arms. "
            "Move or turn to the new operator pose, place both hands where "
            "you want the new neutral pose, then press [c] again."
        )
        logger_mp.info(
            "[Recenter] The second [c] performs a brief automatic neutral "
            "capture, updates the frozen head-yaw frame, and resumes control."
        )

'''

text = text.replace(
    startup_anchor,
    startup_message + startup_anchor,
    1,
)

# ---------------------------------------------------------------------
# 3. Add recenter state beside the auto-axis state.
# ---------------------------------------------------------------------
state_anchor = '''        forward_registration_sample = None
        translation_control_enabled = False
'''

if text.count(state_anchor) != 1:
    fail(
        "Could not uniquely locate the auto-axis runtime state."
    )

state_replacement = state_anchor + '''        recenter_paused = False
        resume_after_recenter = False
'''

text = text.replace(
    state_anchor,
    state_replacement,
    1,
)

# ---------------------------------------------------------------------
# 4. Let calibration completion resume either a loaded map session
#    or a requested recenter.
# ---------------------------------------------------------------------
old_auto_enable = '''                if (
                    translation_map_ready
                    and saved_translation_axis_map is not None
                ):
                    translation_control_enabled = True
                    last_left_command = (
                        robot_left_start_pose.copy()
                    )
                    last_right_command = (
                        robot_right_start_pose.copy()
                    )
                    last_command_time = time.time()

                    logger_mp.info(
                        "[Translation registration] Saved map active; "
                        "control enabled automatically."
                    )
'''

new_auto_enable = '''                if (
                    translation_map_ready
                    and (
                        saved_translation_axis_map is not None
                        or resume_after_recenter
                    )
                ):
                    completed_recenter = resume_after_recenter
                    resume_after_recenter = False
                    translation_control_enabled = True
                    last_left_command = (
                        robot_left_start_pose.copy()
                    )
                    last_right_command = (
                        robot_right_start_pose.copy()
                    )
                    last_command_time = time.time()

                    if completed_recenter:
                        logger_mp.info(
                            "[Recenter] Complete. New hand neutral and "
                            "head-yaw frame are active; control resumed."
                        )
                    else:
                        logger_mp.info(
                            "[Translation registration] Saved map active; "
                            "control enabled automatically."
                        )
'''

if text.count(old_auto_enable) != 1:
    fail(
        "Could not locate the saved-map automatic-enable block."
    )

text = text.replace(
    old_auto_enable,
    new_auto_enable,
    1,
)

# ---------------------------------------------------------------------
# 5. Handle c after valid guarded wrist poses are available and before
#    the normal target is constructed.
# ---------------------------------------------------------------------
control_anchor = (
    "            # AUTO_TRANSLATION_REGISTRATION_V1\n"
)

if text.count(control_anchor) != 1:
    fail(
        "Could not uniquely locate the auto-axis control block."
    )

recenter_logic = '''            # PAUSE_RECENTER_V1
            if RECENTER_TOGGLE:
                RECENTER_TOGGLE = False

                if not arm_calibrated:
                    logger_mp.warning(
                        "[Recenter] Ignored while calibration is active."
                    )

                elif not translation_map_ready:
                    logger_mp.warning(
                        "[Recenter] Ignored because no translation map "
                        "is active yet."
                    )

                elif not recenter_paused:
                    (
                        pause_left_robot_pose,
                        pause_right_robot_pose,
                    ) = get_current_wrist_poses_from_fk(
                        arm_ik,
                        current_lr_arm_q,
                    )

                    robot_left_start_pose = (
                        pause_left_robot_pose.copy()
                    )
                    robot_right_start_pose = (
                        pause_right_robot_pose.copy()
                    )

                    xr_left_start_pose = guarded_left_pose.copy()
                    xr_right_start_pose = guarded_right_pose.copy()

                    last_left_command = (
                        pause_left_robot_pose.copy()
                    )
                    last_right_command = (
                        pause_right_robot_pose.copy()
                    )
                    last_command_time = time.time()

                    translation_control_enabled = False
                    recenter_paused = True

                    logger_mp.info(
                        "[Recenter] PAUSED. Robot arms are holding their "
                        "current measured poses."
                    )
                    logger_mp.info(
                        "[Recenter] Move or turn freely. When ready, make "
                        "both hands visible, place them at the desired new "
                        "neutral pose, hold still, and press [c] again."
                    )

                else:
                    recenter_paused = False
                    resume_after_recenter = True
                    translation_control_enabled = False
                    arm_calibrated = False
                    calibration_ready_frames = 0

                    logger_mp.info(
                        "[Recenter] RESUME requested. Hold both hands and "
                        "head still for the brief automatic capture."
                    )

                    continue

'''

text = text.replace(
    control_anchor,
    recenter_logic + control_anchor,
    1,
)

TARGET.write_text(text)

print(f"Backup preserved: {BACKUP.resolve()}")
print(f"Created:          {TARGET.resolve()}")
print()
print("New control:")
print("  c once  = pause and hold the current measured robot wrist poses")
print("  move/turn and place both hands at the desired new neutral pose")
print("  c again = recapture wrists + current head yaw, then resume")
print("  q       = stop")
