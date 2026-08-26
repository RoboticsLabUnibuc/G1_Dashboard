#!/usr/bin/env python3
"""Shared YOLO annotation helpers for aligned and point-cloud views."""

from __future__ import annotations

import cv2
import numpy as np

PALETTE_BGR = (
    (0, 220, 255),
    (255, 80, 0),
    (0, 255, 100),
    (200, 0, 255),
    (255, 200, 0),
    (0, 150, 255),
    (255, 0, 150),
    (0, 255, 220),
    (180, 255, 0),
    (255, 120, 0),
)


def detection_color(
    detection: dict,
) -> tuple[int, int, int]:
    try:
        class_id = int(
            detection.get("class_id", 0)
        )
    except (TypeError, ValueError):
        class_id = 0

    return PALETTE_BGR[
        class_id % len(PALETTE_BGR)
    ]


def detection_label(
    detection: dict,
) -> str:
    label = str(
        detection.get("label") or "object"
    )

    try:
        confidence = float(
            detection.get("confidence", 0.0)
        )
    except (TypeError, ValueError):
        confidence = 0.0

    return f"{label} {confidence:.0%}"


def normalized_box(
    detection: dict,
    width: int,
    height: int,
) -> tuple[int, int, int, int] | None:
    try:
        x1 = float(detection["x1"])
        y1 = float(detection["y1"])
        x2 = float(detection["x2"])
        y2 = float(detection["y2"])
    except (KeyError, TypeError, ValueError):
        return None

    values = np.asarray(
        (x1, y1, x2, y2),
        dtype=np.float64,
    )

    if not np.all(np.isfinite(values)):
        return None

    x1 = int(round(
        np.clip(x1, 0.0, 1.0) *
        max(0, width - 1)
    ))

    y1 = int(round(
        np.clip(y1, 0.0, 1.0) *
        max(0, height - 1)
    ))

    x2 = int(round(
        np.clip(x2, 0.0, 1.0) *
        max(0, width - 1)
    ))

    y2 = int(round(
        np.clip(y2, 0.0, 1.0) *
        max(0, height - 1)
    ))

    if x2 <= x1 or y2 <= y1:
        return None

    return x1, y1, x2, y2


def draw_aligned_detections(
    frame: np.ndarray,
    detections: list[dict] | tuple[dict, ...],
) -> np.ndarray:
    """Draw normalized RGB detections on any aligned 2-D view."""

    if (
        frame is None
        or frame.ndim != 3
        or not detections
    ):
        return frame

    output = frame.copy()
    height, width = output.shape[:2]

    thickness = max(
        2,
        int(round(min(width, height) / 240.0)),
    )

    font_scale = max(
        0.42,
        min(width, height) / 960.0,
    )

    for detection in detections:
        box = normalized_box(
            detection,
            width,
            height,
        )

        if box is None:
            continue

        x1, y1, x2, y2 = box
        color = detection_color(detection)
        label = detection_label(detection)

        cv2.rectangle(
            output,
            (x1, y1),
            (x2, y2),
            color,
            thickness,
            cv2.LINE_AA,
        )

        (text_width, text_height), baseline = (
            cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                1,
            )
        )

        label_top = max(
            0,
            y1 - text_height - baseline - 6,
        )

        label_bottom = min(
            height - 1,
            label_top +
            text_height +
            baseline +
            6,
        )

        label_right = min(
            width - 1,
            x1 + text_width + 6,
        )

        cv2.rectangle(
            output,
            (x1, label_top),
            (label_right, label_bottom),
            color,
            -1,
        )

        cv2.putText(
            output,
            label,
            (
                x1 + 3,
                label_bottom - baseline - 3,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    return output


def draw_projected_pointcloud_detections(
    canvas: np.ndarray,
    detections: list[dict] | tuple[dict, ...],
    source_pixels: np.ndarray,
    screen_x: np.ndarray,
    screen_y: np.ndarray,
    source_depth_m: np.ndarray,
    source_width: int,
    source_height: int,
) -> np.ndarray:
    """Draw boxes around detected point clusters after virtual projection.

    Each point retains its original aligned RGB coordinate. A detection
    therefore selects its points without running YOLO again. A robust
    foreground-depth filter reduces inclusion of background points inside
    the original 2-D rectangle.
    """

    if (
        canvas is None
        or not detections
        or source_pixels is None
        or screen_x is None
        or screen_y is None
        or source_depth_m is None
    ):
        return canvas

    count = len(source_pixels)

    if (
        count < 1
        or len(screen_x) != count
        or len(screen_y) != count
        or len(source_depth_m) != count
    ):
        return canvas

    pixel_x = source_pixels[:, 0]
    pixel_y = source_pixels[:, 1]

    for detection in detections:
        box = normalized_box(
            detection,
            source_width,
            source_height,
        )

        if box is None:
            continue

        x1, y1, x2, y2 = box

        selected = (
            (pixel_x >= x1) &
            (pixel_x <= x2) &
            (pixel_y >= y1) &
            (pixel_y <= y2) &
            np.isfinite(source_depth_m) &
            (source_depth_m > 0.0)
        )

        if np.count_nonzero(selected) < 4:
            continue

        central_x1 = x1 + int(
            round((x2 - x1) * 0.25)
        )

        central_x2 = x2 - int(
            round((x2 - x1) * 0.25)
        )

        central_y1 = y1 + int(
            round((y2 - y1) * 0.25)
        )

        central_y2 = y2 - int(
            round((y2 - y1) * 0.25)
        )

        central = (
            selected &
            (pixel_x >= central_x1) &
            (pixel_x <= central_x2) &
            (pixel_y >= central_y1) &
            (pixel_y <= central_y2)
        )

        if np.count_nonzero(central) >= 3:
            reference_depth = float(
                np.median(
                    source_depth_m[central]
                )
            )
        else:
            reference_depth = float(
                np.percentile(
                    source_depth_m[selected],
                    30.0,
                )
            )

        tolerance = max(
            0.18,
            reference_depth * 0.22,
        )

        foreground = (
            selected &
            (
                np.abs(
                    source_depth_m -
                    reference_depth
                ) <= tolerance
            )
        )

        if np.count_nonzero(foreground) < 4:
            foreground = selected

        projected_x = screen_x[foreground]
        projected_y = screen_y[foreground]

        if len(projected_x) < 4:
            continue

        left = int(round(
            np.percentile(projected_x, 2.0)
        ))

        right = int(round(
            np.percentile(projected_x, 98.0)
        ))

        top = int(round(
            np.percentile(projected_y, 2.0)
        ))

        bottom = int(round(
            np.percentile(projected_y, 98.0)
        ))

        canvas_height, canvas_width = (
            canvas.shape[:2]
        )

        left = int(np.clip(
            left,
            0,
            canvas_width - 1,
        ))

        right = int(np.clip(
            right,
            0,
            canvas_width - 1,
        ))

        top = int(np.clip(
            top,
            0,
            canvas_height - 1,
        ))

        bottom = int(np.clip(
            bottom,
            0,
            canvas_height - 1,
        ))

        if right - left < 4:
            left = max(0, left - 2)
            right = min(
                canvas_width - 1,
                right + 2,
            )

        if bottom - top < 4:
            top = max(0, top - 2)
            bottom = min(
                canvas_height - 1,
                bottom + 2,
            )

        color = detection_color(detection)
        label = detection_label(detection)

        cv2.rectangle(
            canvas,
            (left, top),
            (right, bottom),
            color,
            2,
            cv2.LINE_AA,
        )

        label_y = max(16, top - 5)

        cv2.putText(
            canvas,
            label,
            (left, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            color,
            1,
            cv2.LINE_AA,
        )

    return canvas
