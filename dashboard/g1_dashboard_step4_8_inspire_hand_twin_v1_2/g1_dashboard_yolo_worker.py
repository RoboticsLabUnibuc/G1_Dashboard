#!/usr/bin/env python3
"""Isolated latest-frame YOLO worker.

Input protocol on stdin:
    <4sIIIQ little-endian header>
    magic, width, height, byte_count, sequence
    followed by width * height * 3 raw BGR bytes.

Output protocol on stdout:
    One compact JSON object per processed frame.
"""

from __future__ import annotations

import argparse
import json
import statistics
import struct
import sys
import time
from pathlib import Path

import numpy as np
import torch
from ultralytics import YOLO

FRAME_HEADER = struct.Struct("<4sIIIQ")
FRAME_MAGIC = b"G1YF"

MAX_WIDTH = 1920
MAX_HEIGHT = 1080
MAX_FRAME_BYTES = MAX_WIDTH * MAX_HEIGHT * 3


def log(message: str) -> None:
    print(
        f"[G1 YOLO worker] {message}",
        file=sys.stderr,
        flush=True,
    )


def configure_ultralytics_logging() -> None:
    try:
        from ultralytics.utils import LOGGER

        for handler in LOGGER.handlers:
            if hasattr(handler, "setStream"):
                handler.setStream(sys.stderr)
            else:
                handler.stream = sys.stderr
    except Exception:
        pass


def read_exact(stream, length: int) -> bytes | None:
    chunks = bytearray()

    while len(chunks) < length:
        chunk = stream.read(length - len(chunks))

        if not chunk:
            return None

        chunks.extend(chunk)

    return bytes(chunks)


def emit(payload: dict) -> None:
    sys.stdout.write(
        json.dumps(
            payload,
            separators=(",", ":"),
            allow_nan=False,
        )
        + "\n"
    )
    sys.stdout.flush()


class Detector:
    def __init__(
        self,
        model_path: Path,
        confidence: float,
        iou: float,
        image_size: int,
        maximum_detections: int,
    ) -> None:
        if not model_path.is_file():
            raise FileNotFoundError(
                f"YOLO model not found: {model_path}"
            )

        self.confidence = confidence
        self.iou = iou
        self.image_size = image_size
        self.maximum_detections = maximum_detections

        self.cuda = bool(torch.cuda.is_available())
        self.device = 0 if self.cuda else "cpu"
        self.quantize = "fp16" if self.cuda else None

        torch.set_grad_enabled(False)

        if self.cuda:
            torch.backends.cudnn.benchmark = True

        started = time.perf_counter()
        self.model = YOLO(str(model_path))

        # Warm up the selected backend without accessing a camera.
        warmup = np.zeros(
            (480, 640, 3),
            dtype=np.uint8,
        )

        for _ in range(2):
            self._predict_raw(warmup)

        if self.cuda:
            torch.cuda.synchronize()

        startup_ms = (
            time.perf_counter() - started
        ) * 1000.0

        log(
            f"ready model={model_path} "
            f"device={self.device} "
            f"precision={'fp16' if self.cuda else 'fp32'} "
            f"startup_ms={startup_ms:.1f}"
        )

    def _predict_raw(self, frame: np.ndarray):
        arguments = {
            "source": frame,
            "imgsz": self.image_size,
            "conf": self.confidence,
            "iou": self.iou,
            "max_det": self.maximum_detections,
            "device": self.device,
            "verbose": False,
            "save": False,
        }

        if self.quantize is not None:
            arguments["quantize"] = self.quantize

        return self.model.predict(**arguments)[0]

    def detect(
        self,
        frame: np.ndarray,
        sequence: int,
    ) -> dict:
        if self.cuda:
            torch.cuda.synchronize()

        started = time.perf_counter()

        with torch.inference_mode():
            result = self._predict_raw(frame)

        if self.cuda:
            torch.cuda.synchronize()

        inference_ms = (
            time.perf_counter() - started
        ) * 1000.0

        height, width = frame.shape[:2]
        detections: list[dict] = []

        boxes = result.boxes

        if boxes is not None and len(boxes) > 0:
            xyxy = (
                boxes.xyxy
                .detach()
                .cpu()
                .numpy()
            )

            confidences = (
                boxes.conf
                .detach()
                .cpu()
                .numpy()
            )

            classes = (
                boxes.cls
                .detach()
                .cpu()
                .numpy()
                .astype(np.int32)
            )

            names = result.names

            for coordinates, confidence, class_id in zip(
                xyxy,
                confidences,
                classes,
            ):
                x1, y1, x2, y2 = map(
                    float,
                    coordinates,
                )

                if isinstance(names, dict):
                    label = str(
                        names.get(
                            int(class_id),
                            class_id,
                        )
                    )
                elif 0 <= int(class_id) < len(names):
                    label = str(names[int(class_id)])
                else:
                    label = str(class_id)

                detections.append(
                    {
                        "class_id": int(class_id),
                        "label": label,
                        "confidence": round(
                            float(confidence),
                            5,
                        ),
                        "x1": round(
                            max(0.0, min(1.0, x1 / width)),
                            6,
                        ),
                        "y1": round(
                            max(0.0, min(1.0, y1 / height)),
                            6,
                        ),
                        "x2": round(
                            max(0.0, min(1.0, x2 / width)),
                            6,
                        ),
                        "y2": round(
                            max(0.0, min(1.0, y2 / height)),
                            6,
                        ),
                    }
                )

        return {
            "schema": "g1_dashboard.yolo.v1",
            "ok": True,
            "sequence": int(sequence),
            "completed_unix_ns": time.time_ns(),
            "width": int(width),
            "height": int(height),
            "inference_ms": round(inference_ms, 3),
            "detections": detections,
        }


def self_test(detector: Detector) -> int:
    frame = np.zeros(
        (480, 640, 3),
        dtype=np.uint8,
    )

    times = []

    for sequence in range(5):
        result = detector.detect(
            frame,
            sequence,
        )

        times.append(
            float(result["inference_ms"])
        )

    emit(
        {
            "self_test": "PASS",
            "samples": len(times),
            "median_ms": round(
                statistics.median(times),
                3,
            ),
            "maximum_ms": round(
                max(times),
                3,
            ),
            "cuda": detector.cuda,
            "device": str(detector.device),
            "precision":
                "fp16" if detector.cuda else "fp32",
        }
    )

    return 0


def worker_loop(detector: Detector) -> int:
    input_stream = sys.stdin.buffer

    while True:
        header = read_exact(
            input_stream,
            FRAME_HEADER.size,
        )

        if header is None:
            return 0

        try:
            (
                magic,
                width,
                height,
                byte_count,
                sequence,
            ) = FRAME_HEADER.unpack(header)
        except struct.error as exc:
            log(f"invalid frame header: {exc}")
            return 2

        expected_bytes = width * height * 3

        if (
            magic != FRAME_MAGIC
            or width < 1
            or height < 1
            or width > MAX_WIDTH
            or height > MAX_HEIGHT
            or byte_count != expected_bytes
            or byte_count > MAX_FRAME_BYTES
        ):
            log(
                "invalid frame declaration "
                f"magic={magic!r} "
                f"size={width}x{height} "
                f"bytes={byte_count}"
            )
            return 2

        payload = read_exact(
            input_stream,
            byte_count,
        )

        if payload is None:
            return 0

        try:
            frame = (
                np.frombuffer(
                    payload,
                    dtype=np.uint8,
                )
                .reshape(
                    height,
                    width,
                    3,
                )
            )

            emit(
                detector.detect(
                    frame,
                    sequence,
                )
            )
        except Exception as exc:
            emit(
                {
                    "schema": "g1_dashboard.yolo.v1",
                    "ok": False,
                    "sequence": int(sequence),
                    "error":
                        f"{type(exc).__name__}: {exc}",
                }
            )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--model",
        required=True,
    )

    parser.add_argument(
        "--confidence",
        type=float,
        default=0.35,
    )

    parser.add_argument(
        "--iou",
        type=float,
        default=0.50,
    )

    parser.add_argument(
        "--image-size",
        type=int,
        default=640,
    )

    parser.add_argument(
        "--maximum-detections",
        type=int,
        default=50,
    )

    parser.add_argument(
        "--self-test",
        action="store_true",
    )

    return parser.parse_args()


def main() -> int:
    configure_ultralytics_logging()
    arguments = parse_arguments()

    detector = Detector(
        model_path=Path(
            arguments.model
        ).expanduser(),
        confidence=max(
            0.01,
            min(0.99, arguments.confidence),
        ),
        iou=max(
            0.01,
            min(0.99, arguments.iou),
        ),
        image_size=max(
            320,
            min(1280, arguments.image_size),
        ),
        maximum_detections=max(
            1,
            min(300, arguments.maximum_detections),
        ),
    )

    if arguments.self_test:
        return self_test(detector)

    return worker_loop(detector)


if __name__ == "__main__":
    raise SystemExit(main())
