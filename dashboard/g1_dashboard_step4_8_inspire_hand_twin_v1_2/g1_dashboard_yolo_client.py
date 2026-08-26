#!/usr/bin/env python3
"""Non-blocking latest-frame client for the isolated G1 YOLO worker."""

from __future__ import annotations

import atexit
import json
import os
import struct
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

FRAME_HEADER = struct.Struct("<4sIIIQ")
FRAME_MAGIC = b"G1YF"


class YoloClient:
    def __init__(
        self,
        logger,
        worker_python: Path,
        worker_script: Path,
        model_path: Path,
        inference_hz: float = 15.0,
        confidence: float = 0.35,
        iou: float = 0.50,
        image_size: int = 640,
        maximum_detections: int = 50,
    ) -> None:
        self.logger = logger
        self.worker_python = worker_python
        self.worker_script = worker_script
        self.model_path = model_path

        self.frame_period = (
            1.0 /
            max(1.0, min(30.0, inference_hz))
        )

        self.confidence = max(
            0.01,
            min(0.99, confidence),
        )

        self.iou = max(
            0.01,
            min(0.99, iou),
        )

        self.image_size = max(
            320,
            min(1280, image_size),
        )

        self.maximum_detections = max(
            1,
            min(300, maximum_detections),
        )

        self.condition = threading.Condition()

        self.pending_frame = None
        self.latest_result = None
        self.latest_result_monotonic = 0.0

        self.process = None
        self.stopping = False
        self.state = "STARTING"
        self.last_error = None

        self.sequence = (
            time.time_ns() &
            0xFFFFFFFFFFFFFFFF
        )

        self.next_submit_time = 0.0
        self.accepted_frames = 0
        self.completed_frames = 0
        self.replaced_frames = 0

        self.thread = threading.Thread(
            target=self._supervisor,
            name="g1-yolo-client",
            daemon=True,
        )

        self.thread.start()
        atexit.register(self.close)

    def submit(
        self,
        frame: np.ndarray,
    ) -> bool:
        if (
            frame is None
            or not isinstance(frame, np.ndarray)
            or frame.ndim != 3
            or frame.shape[2] != 3
        ):
            return False

        now = time.monotonic()

        with self.condition:
            if (
                self.stopping
                or now < self.next_submit_time
            ):
                return False

            self.next_submit_time = (
                now + self.frame_period
            )

            self.sequence = (
                self.sequence + 1
            ) & 0xFFFFFFFFFFFFFFFF

            copied = np.array(
                frame,
                dtype=np.uint8,
                copy=True,
                order="C",
            )

            if self.pending_frame is not None:
                self.replaced_frames += 1

            self.pending_frame = (
                self.sequence,
                copied,
            )

            self.accepted_frames += 1
            self.condition.notify_all()

        return True

    def latest(
        self,
        maximum_age_seconds: float = 0.40,
    ) -> dict | None:
        now = time.monotonic()

        with self.condition:
            if self.latest_result is None:
                return None

            if (
                now -
                self.latest_result_monotonic >
                maximum_age_seconds
            ):
                return None

            return self.latest_result

    def status(self) -> dict:
        with self.condition:
            result = self.latest_result

            return {
                "state": self.state,
                "worker_python":
                    str(self.worker_python),
                "worker_script":
                    str(self.worker_script),
                "model": str(self.model_path),
                "inference_hz":
                    round(1.0 / self.frame_period, 2),
                "accepted_frames":
                    self.accepted_frames,
                "completed_frames":
                    self.completed_frames,
                "replaced_frames":
                    self.replaced_frames,
                "latest_inference_ms":
                    (
                        result.get("inference_ms")
                        if isinstance(result, dict)
                        else None
                    ),
                "latest_detection_count":
                    (
                        len(result.get("detections", []))
                        if isinstance(result, dict)
                        else 0
                    ),
                "last_error": self.last_error,
            }

    def close(self) -> None:
        with self.condition:
            if self.stopping:
                return

            self.stopping = True
            self.pending_frame = None

            process = self.process
            self.condition.notify_all()

        if process is not None:
            try:
                process.terminate()
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                    process.wait(timeout=1.0)
                except Exception:
                    pass
            except Exception:
                pass

        if (
            self.thread.is_alive()
            and threading.current_thread()
            is not self.thread
        ):
            self.thread.join(timeout=4.0)

        with self.condition:
            self.process = None
            self.state = "STOPPED"

    def _worker_environment(self) -> dict[str, str]:
        environment = os.environ.copy()

        for key in (
            "PYTHONHOME",
            "PYTHONPATH",
            "PYTHONNOUSERSITE",
            "CONDA_PREFIX",
            "CONDA_DEFAULT_ENV",
            "CONDA_PROMPT_MODIFIER",
        ):
            environment.pop(key, None)

        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        environment.setdefault(
            "CUDA_MODULE_LOADING",
            "LAZY",
        )

        return environment

    def _worker_command(self) -> list[str]:
        return [
            str(self.worker_python),
            "-u",
            str(self.worker_script),
            "--model",
            str(self.model_path),
            "--confidence",
            str(self.confidence),
            "--iou",
            str(self.iou),
            "--image-size",
            str(self.image_size),
            "--maximum-detections",
            str(self.maximum_detections),
        ]

    def _supervisor(self) -> None:
        while True:
            with self.condition:
                if self.stopping:
                    return

            try:
                self._run_process()
            except Exception as exc:
                with self.condition:
                    if self.stopping:
                        return

                    self.state = "ERROR"
                    self.last_error = (
                        f"{type(exc).__name__}: {exc}"
                    )

                self.logger.warning(
                    "[G1 YOLO client] worker failure: %s",
                    self.last_error,
                )

            with self.condition:
                if self.stopping:
                    return

                self.condition.wait(timeout=2.0)

    def _run_process(self) -> None:
        process = subprocess.Popen(
            self._worker_command(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            env=self._worker_environment(),
            start_new_session=False,
            close_fds=True,
            bufsize=0,
        )

        with self.condition:
            if self.stopping:
                process.terminate()
                return

            self.process = process
            self.state = "STARTING"
            self.last_error = None

        self.logger.info(
            "[G1 YOLO client] launched pid=%d model=%s",
            process.pid,
            self.model_path,
        )

        try:
            while True:
                with self.condition:
                    while (
                        self.pending_frame is None
                        and not self.stopping
                    ):
                        self.condition.wait(
                            timeout=0.5
                        )

                        if process.poll() is not None:
                            raise RuntimeError(
                                "worker exited with code "
                                f"{process.returncode}"
                            )

                    if self.stopping:
                        return

                    sequence, frame = (
                        self.pending_frame
                    )

                    self.pending_frame = None

                if (
                    process.stdin is None
                    or process.stdout is None
                ):
                    raise RuntimeError(
                        "worker pipes unavailable"
                    )

                height, width = frame.shape[:2]

                header = FRAME_HEADER.pack(
                    FRAME_MAGIC,
                    width,
                    height,
                    int(frame.nbytes),
                    sequence,
                )

                process.stdin.write(header)
                process.stdin.write(
                    memoryview(frame).cast("B")
                )
                process.stdin.flush()

                response_line = (
                    process.stdout.readline()
                )

                if not response_line:
                    raise RuntimeError(
                        "worker closed its response pipe"
                    )

                response = json.loads(
                    response_line.decode("utf-8")
                )

                if not isinstance(response, dict):
                    raise RuntimeError(
                        "worker response is not an object"
                    )

                if not response.get("ok"):
                    raise RuntimeError(
                        response.get(
                            "error",
                            "worker reported failure",
                        )
                    )

                if (
                    int(response.get("sequence", -1))
                    != sequence
                ):
                    raise RuntimeError(
                        "worker response sequence mismatch"
                    )

                with self.condition:
                    self.latest_result = response
                    self.latest_result_monotonic = (
                        time.monotonic()
                    )
                    self.completed_frames += 1
                    self.state = "RUNNING"
                    self.last_error = None
        finally:
            with self.condition:
                if self.process is process:
                    self.process = None

            for stream in (
                process.stdin,
                process.stdout,
            ):
                try:
                    if stream is not None:
                        stream.close()
                except Exception:
                    pass

            if process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=2.0)
                except Exception:
                    try:
                        process.kill()
                    except Exception:
                        pass


def create_yolo_client(
    logger,
) -> YoloClient | None:
    enabled = os.environ.get(
        "G1_DASHBOARD_YOLO_AVAILABLE",
        "1",
    ).strip().lower() not in {
        "0",
        "false",
        "no",
        "off",
        "disabled",
    }

    if not enabled:
        logger.info(
            "[G1 YOLO client] disabled by environment"
        )
        return None

    dashboard_dir = Path(__file__).resolve().parent

    worker_python = Path(
        os.environ.get(
            "G1_DASHBOARD_YOLO_PYTHON",
            "/usr/bin/python3",
        )
    ).expanduser()

    worker_script = Path(
        os.environ.get(
            "G1_DASHBOARD_YOLO_WORKER",
            str(
                dashboard_dir /
                "g1_dashboard_yolo_worker.py"
            ),
        )
    ).expanduser()

    model_path = Path(
        os.environ.get(
            "G1_DASHBOARD_YOLO_MODEL",
            str(
                dashboard_dir /
                "models/yolov8n.pt"
            ),
        )
    ).expanduser()

    required = (
        worker_python,
        worker_script,
        model_path,
    )

    missing = [
        str(path)
        for path in required
        if not path.is_file()
    ]

    if missing:
        logger.warning(
            "[G1 YOLO client] unavailable; missing=%s",
            ",".join(missing),
        )
        return None

    try:
        return YoloClient(
            logger=logger,
            worker_python=worker_python,
            worker_script=worker_script,
            model_path=model_path,
            inference_hz=float(
                os.environ.get(
                    "G1_DASHBOARD_YOLO_HZ",
                    "15",
                )
            ),
            confidence=float(
                os.environ.get(
                    "G1_DASHBOARD_YOLO_CONFIDENCE",
                    "0.35",
                )
            ),
            iou=float(
                os.environ.get(
                    "G1_DASHBOARD_YOLO_IOU",
                    "0.50",
                )
            ),
            image_size=int(
                os.environ.get(
                    "G1_DASHBOARD_YOLO_IMAGE_SIZE",
                    "640",
                )
            ),
            maximum_detections=int(
                os.environ.get(
                    "G1_DASHBOARD_YOLO_MAX_DETECTIONS",
                    "50",
                )
            ),
        )
    except Exception as exc:
        logger.warning(
            "[G1 YOLO client] setup failed: %s",
            exc,
        )
        return None
