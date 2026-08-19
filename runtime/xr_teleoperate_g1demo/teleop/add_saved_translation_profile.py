#!/usr/bin/env python3
from pathlib import Path
import shutil

SOURCE = Path("teleop_arm_only_real_gain100_autoaxes.py")
TARGET = Path("teleop_arm_only_real_gain100_autoaxes_saved.py")
BACKUP = Path("teleop_arm_only_real_gain100_autoaxes.WORKING_BACKUP.py")
MARKER = "SAVED_TRANSLATION_PROFILE_V1"


def fail(message: str) -> None:
    raise SystemExit(f"ERROR: {message}")


if not SOURCE.exists():
    fail(f"Missing working source file: {SOURCE.resolve()}")

source_text = SOURCE.read_text()

if "AUTO_TRANSLATION_REGISTRATION_V1" not in source_text:
    fail(
        "The source file does not contain AUTO_TRANSLATION_REGISTRATION_V1. "
        "Use the working auto-axis file that accepts f, o, and g."
    )

if MARKER in source_text:
    fail("The source file already contains the saved-profile patch.")

if not BACKUP.exists():
    shutil.copy2(SOURCE, BACKUP)

shutil.copy2(SOURCE, TARGET)
text = TARGET.read_text()

# ---------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------
if "import json\n" not in text:
    import_anchor = "import argparse\n"
    if import_anchor not in text:
        fail("Could not locate 'import argparse'.")
    text = text.replace(
        import_anchor,
        import_anchor + "import json\nfrom pathlib import Path\n",
        1,
    )

# ---------------------------------------------------------------------
# Profile load/save helpers
# ---------------------------------------------------------------------
main_anchor = "\nif __name__ == '__main__':\n"
if text.count(main_anchor) != 1:
    fail("Could not uniquely locate the main-program anchor.")

helpers = r'''

# SAVED_TRANSLATION_PROFILE_V1
TRANSLATION_PROFILE_VERSION = 1


def load_translation_profile(profile_path):
    # Load and validate the saved horizontal translation map.
    path = Path(profile_path).expanduser()

    if not path.exists():
        return None

    try:
        payload = json.loads(path.read_text())
        version = int(payload.get("version", -1))

        if version != TRANSLATION_PROFILE_VERSION:
            raise ValueError(
                f"unsupported profile version {version}"
            )

        matrix = np.asarray(
            payload["translation_axis_map"],
            dtype=float,
        )

        if matrix.shape != (3, 3):
            raise ValueError(
                f"expected a 3x3 matrix, got {matrix.shape}"
            )

        if not np.all(np.isfinite(matrix)):
            raise ValueError("matrix contains non-finite values")

        if not np.allclose(
            matrix[2, :],
            np.array([0.0, 0.0, 1.0]),
            atol=1e-6,
        ):
            raise ValueError(
                "profile does not preserve vertical translation"
            )

        if not np.allclose(
            matrix[:2, 2],
            np.zeros(2),
            atol=1e-6,
        ):
            raise ValueError(
                "profile mixes vertical motion into horizontal motion"
            )

        horizontal_det = float(
            np.linalg.det(matrix[:2, :2])
        )

        if (
            not np.isfinite(horizontal_det)
            or abs(horizontal_det) < 0.20
        ):
            raise ValueError(
                "horizontal mapping is singular or nearly singular"
            )

        return matrix

    except Exception as exc:
        raise RuntimeError(
            f"Could not load translation profile {path}: {exc}"
        ) from exc


def save_translation_profile(profile_path, translation_axis_map):
    # Atomically save the learned horizontal translation map.
    path = Path(profile_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)

    matrix = np.asarray(
        translation_axis_map,
        dtype=float,
    )

    payload = {
        "version": TRANSLATION_PROFILE_VERSION,
        "translation_axis_map": matrix.tolist(),
    }

    temporary_path = path.with_name(
        path.name + ".tmp"
    )

    temporary_path.write_text(
        json.dumps(
            payload,
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )

    temporary_path.replace(path)
'''

text = text.replace(main_anchor, helpers + main_anchor, 1)

# ---------------------------------------------------------------------
# CLI options
# ---------------------------------------------------------------------
parse_anchor = "    args = parser.parse_args()\n"
if text.count(parse_anchor) != 1:
    fail("Could not uniquely locate parser.parse_args().")

profile_args = r'''    parser.add_argument(
        '--translation-profile',
        type=str,
        default='~/.config/xr_teleoperate/g1_translation_profile.json',
        help='Path used to load and save the learned horizontal translation map',
    )
    parser.add_argument(
        '--ignore-translation-profile',
        action='store_true',
        help='Ignore the saved map and perform f/o/g registration again',
    )

'''

text = text.replace(
    parse_anchor,
    profile_args + parse_anchor,
    1,
)

profile_init = r'''    translation_profile_path = Path(
        args.translation_profile
    ).expanduser()

    if args.ignore_translation_profile:
        saved_translation_axis_map = None
    else:
        saved_translation_axis_map = load_translation_profile(
            translation_profile_path
        )

'''

text = text.replace(
    parse_anchor,
    parse_anchor + profile_init,
    1,
)

# ---------------------------------------------------------------------
# Print instructions before the user presses r.
# ---------------------------------------------------------------------
ready_anchor = (
    '        logger_mp.info("----------------------------------------------------------------")\n'
)

if text.count(ready_anchor) != 1:
    fail("Could not locate the startup instruction block.")

early_instructions = r'''        if saved_translation_axis_map is None:
            logger_mp.info(
                "[Translation registration] No saved map is active."
            )
            logger_mp.info(
                "[Translation registration] After pressing [r] and "
                "neutral calibration: move BOTH hands forward and press [f]; "
                "return neutral; spread BOTH hands outward and press [o]; "
                "return neutral and press [g]."
            )
            logger_mp.info(
                "[Translation registration] A successful [o] capture "
                f"will be saved to: {translation_profile_path}"
            )
        else:
            logger_mp.info(
                "[Translation registration] Saved map loaded from: "
                f"{translation_profile_path}"
            )
            logger_mp.info(
                "[Translation registration] After pressing [r], neutral "
                "calibration and control engagement are automatic. "
                "The f/o/g sequence is not required."
            )

'''

text = text.replace(
    ready_anchor,
    early_instructions + ready_anchor,
    1,
)

# ---------------------------------------------------------------------
# Initialize registration state from the saved profile.
# ---------------------------------------------------------------------
old_state = r'''        # AUTO_TRANSLATION_REGISTRATION_V1 state.
        translation_axis_map = np.eye(3, dtype=float)
        forward_registration_sample = None
        translation_map_ready = False
        translation_control_enabled = False

        logger_mp.info(
            "[Translation registration] After neutral calibration, "
            "move BOTH hands straight forward at least 10 cm, "
            "hold them, and press [f]."
        )
'''

new_state = r'''        # AUTO_TRANSLATION_REGISTRATION_V1 state.
        if saved_translation_axis_map is None:
            translation_axis_map = np.eye(3, dtype=float)
            translation_map_ready = False
        else:
            translation_axis_map = (
                saved_translation_axis_map.copy()
            )
            translation_map_ready = True

        forward_registration_sample = None
        translation_control_enabled = False
'''

if text.count(old_state) != 1:
    fail(
        "Could not locate the exact auto-axis registration-state block. "
        "The working file may differ from the expected version."
    )

text = text.replace(old_state, new_state, 1)

# ---------------------------------------------------------------------
# Save the map after a successful outward capture.
# ---------------------------------------------------------------------
map_ready_anchor = r'''                        translation_map_ready = True

                        logger_mp.info(
                            "[Translation registration] Horizontal map ready."
                        )
'''

if text.count(map_ready_anchor) != 1:
    fail("Could not locate the successful map-capture block.")

save_block = r'''                        translation_map_ready = True

                        save_translation_profile(
                            translation_profile_path,
                            translation_axis_map,
                        )

                        logger_mp.info(
                            "[Translation registration] Horizontal map ready."
                        )

                        logger_mp.info(
                            "[Translation registration] Saved map to: "
                            f"{translation_profile_path}"
                        )
'''

text = text.replace(
    map_ready_anchor,
    save_block,
    1,
)

# ---------------------------------------------------------------------
# Auto-enable control after neutral calibration when a profile is loaded.
# ---------------------------------------------------------------------
neutral_log = r'''                logger_mp.info(
                    "[Arm calibration] Current operator pose is neutral."
                )
'''

if text.count(neutral_log) != 1:
    fail("Could not locate the neutral-calibration completion message.")

auto_enable = neutral_log + r'''

                if (
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

text = text.replace(
    neutral_log,
    auto_enable,
    1,
)

TARGET.write_text(text)

print(f"Backup preserved: {BACKUP.resolve()}")
print(f"Created:          {TARGET.resolve()}")
print()
print("First registration:")
print("  launch with --ignore-translation-profile")
print("  press r, then f, o, g as instructed")
print("  the map is saved automatically after o")
print()
print("Later sessions:")
print("  launch without --ignore-translation-profile")
print("  press r only")
