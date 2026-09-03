#!/usr/bin/env python3
"""Latest-only, MTU-safe, subscription-driven multi-view JPEG-over-UDP sender."""

from __future__ import annotations

import atexit
import os
import socket
import struct
import threading
import time

import cv2
import numpy as np

# Existing 20-byte packet header. Bit 0 of flags marks the final chunk;
# bits 1..7 carry the view id. Version remains 1 for RGB-only compatibility.
_PACKET_HEADER = struct.Struct("<4sBBIHHHI")
_PACKET_MAGIC = b"G1JP"
_PACKET_VERSION = 1

VIEW_IDS = {
    "rgb": 0,
    "depth": 1,
    "overlay": 2,
    "near": 3,
    "disparity": 4,
    "pointcloud": 5,
    "topdown": 6,
    "lifecam": 7,
}
VIEW_NAMES = tuple(VIEW_IDS)

# Quest -> robot heartbeat: magic, version, enabled-view bit mask.
_CONTROL_PACKET = struct.Struct("<4sBH")
_CONTROL_MAGIC = b"G1QS"
_CONTROL_VERSION = 1

# Bits 0..6 remain the requested camera-view mask. Bit 15 is an
# independent Quest request for the robot's single shared YOLO pipeline.
_CONTROL_YOLO_BIT = 1 << 15
_CONTROL_VIEW_MASK = (1 << len(VIEW_NAMES)) - 1
_CONTROL_ALLOWED_MASK = _CONTROL_VIEW_MASK | _CONTROL_YOLO_BIT


def _env_enabled(name: str, default: bool) -> bool:
    raw = os.environ.get(name, "1" if default else "0").strip().lower()
    return raw not in {"0", "false", "no", "off", "disabled"}


def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
    value = int(os.environ.get(name, str(default)))
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    value = float(os.environ.get(name, str(default)))
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


class QuestUdpJpegSender:
    """Encode/send only the newest requested view batch on background threads."""

    def __init__(
        self,
        logger,
        host: str,
        port: int,
        control_port: int,
        subscription_timeout: float,
        fps: int,
        jpeg_quality: int,
        packet_bytes: int,
        max_jpeg_bytes: int,
        view_names: tuple[str, ...],
        fallback_mask: int,
    ):
        if packet_bytes <= _PACKET_HEADER.size:
            raise ValueError("UDP packet size is smaller than its header")

        selected_views = tuple(view_names)
        if not selected_views:
            raise ValueError("Quest UDP sender requires at least one view")

        unknown_views = set(selected_views).difference(VIEW_IDS)
        if unknown_views:
            raise ValueError(
                f"unknown Quest UDP views: {sorted(unknown_views)}"
            )

        self._view_names = selected_views
        self._fallback_mask = (
            int(fallback_mask) & _CONTROL_VIEW_MASK
        )
        self._logger = logger
        self._target = socket.getaddrinfo(
            host, port, socket.AF_INET, socket.SOCK_DGRAM
        )[0][4]
        self._fps = fps
        self._jpeg_quality = jpeg_quality
        self._payload_bytes = packet_bytes - _PACKET_HEADER.size
        self._max_jpeg_bytes = max_jpeg_bytes
        self._frame_period = 1.0 / float(fps)
        self._subscription_timeout = subscription_timeout

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1_048_576)
        except OSError:
            pass

        self._control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._control_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._control_socket.bind(("0.0.0.0", control_port))
        self._control_socket.settimeout(0.5)

        self._condition = threading.Condition()
        self._pending_frames: dict[str, np.ndarray] = {}
        self._stopping = False
        self._next_accept_time = {
            name: 0.0 for name in self._view_names
        }
        self._frame_id = int(time.time_ns()) & 0xFFFFFFFF
        self._subscription_mask = self._fallback_mask
        self._last_subscription_time = 0.0
        self._quest_yolo_requested = False
        self._last_logged_mask: int | None = None
        self._last_logged_yolo: bool | None = None
        self._replaced_frames = 0
        self._frames_sent = {
            name: 0 for name in self._view_names
        }
        self._last_sizes: dict[str, tuple[int, int]] = {}
        self._next_stats_time = time.monotonic() + 5.0
        self._next_warning_time = 0.0

        self._thread = threading.Thread(
            target=self._worker, name="g1-quest-udp", daemon=True
        )
        self._control_thread = threading.Thread(
            target=self._control_worker,
            name="g1-quest-udp-control",
            daemon=True,
        )
        self._thread.start()
        self._control_thread.start()
        atexit.register(self.close)

        self._logger.info(
            "[G1 Quest UDP] enabled target=%s:%d control=0.0.0.0:%d "
            "views=%s fps=%d quality=%d "
            "packet_bytes=%d header_bytes=%d",
            self._target[0],
            self._target[1],
            control_port,
            ",".join(self._view_names),
            self._fps,
            self._jpeg_quality,
            packet_bytes,
            _PACKET_HEADER.size,
        )

    def _current_mask_locked(self, now: float) -> int:
        expired = (
            self._last_subscription_time <= 0.0
            or now - self._last_subscription_time
            > self._subscription_timeout
        )
        return (
            self._fallback_mask
            if expired
            else self._subscription_mask
        )

    def requested_modes(self) -> tuple[str, ...]:
        now = time.monotonic()
        with self._condition:
            mask = self._current_mask_locked(now)

        return tuple(
            name
            for name in self._view_names
            if mask & (1 << VIEW_IDS[name])
        )

    def requested_modes_due(self) -> tuple[str, ...]:
        """Return requested views whose independent clocks are due."""
        now = time.monotonic()
        early_tolerance = min(
            0.005,
            self._frame_period * 0.25,
        )

        with self._condition:
            if self._stopping:
                return ()

            mask = self._current_mask_locked(now)

            return tuple(
                name
                for name in self._view_names
                if mask & (1 << VIEW_IDS[name])
                and (
                    self._next_accept_time[name] <= 0.0
                    or now + early_tolerance
                    >= self._next_accept_time[name]
                )
            )

    def yolo_requested(self) -> bool:
        """Return the live Quest YOLO request.

        The request expires with the normal subscription heartbeat, so a
        disconnected or suspended headset cannot leave inference enabled.
        """
        now = time.monotonic()
        with self._condition:
            expired = (
                self._last_subscription_time <= 0.0
                or now - self._last_subscription_time
                > self._subscription_timeout
            )
            return bool(
                not expired
                and self._quest_yolo_requested
            )

    def submit_views(self, frames: dict[str, np.ndarray]) -> None:
        """Merge frames into independent latest-only view slots."""
        if not frames:
            return

        try:
            now = time.monotonic()
            early_tolerance = min(
                0.005,
                self._frame_period * 0.25,
            )

            with self._condition:
                if self._stopping:
                    return

                active_mask = self._current_mask_locked(now)
                accepted = False

                for name in self._view_names:
                    frame = frames.get(name)

                    if (
                        frame is None
                        or not active_mask
                        & (1 << VIEW_IDS[name])
                    ):
                        continue

                    deadline = self._next_accept_time[name]

                    if (
                        deadline > 0.0
                        and now + early_tolerance < deadline
                    ):
                        continue

                    copied = np.array(
                        frame,
                        dtype=np.uint8,
                        copy=True,
                        order="C",
                    )

                    if name in self._pending_frames:
                        self._replaced_frames += 1

                    self._pending_frames[name] = copied
                    accepted = True

                    if deadline <= 0.0:
                        next_deadline = (
                            now + self._frame_period
                        )
                    else:
                        next_deadline = (
                            deadline + self._frame_period
                        )

                        if next_deadline <= now:
                            missed = (
                                int(
                                    (now - next_deadline)
                                    / self._frame_period
                                )
                                + 1
                            )
                            next_deadline += (
                                missed * self._frame_period
                            )

                    self._next_accept_time[name] = (
                        next_deadline
                    )

                if accepted:
                    self._condition.notify()

        except Exception as exc:
            self._warn(f"frame submission failed: {exc}")

    def submit(self, bgr: np.ndarray) -> None:
        """Compatibility with the original RGB-only runner."""
        self.submit_views({"rgb": bgr})

    def close(self) -> None:
        with self._condition:
            if self._stopping:
                return
            self._stopping = True
            self._pending_frames.clear()
            self._condition.notify_all()
        for sock in (self._control_socket, self._socket):
            try:
                sock.close()
            except OSError:
                pass

    def _control_worker(self) -> None:
        while True:
            with self._condition:
                if self._stopping:
                    return
            try:
                packet, remote = self._control_socket.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                with self._condition:
                    if self._stopping:
                        return
                continue

            if remote[0] != self._target[0] or len(packet) != _CONTROL_PACKET.size:
                continue
            try:
                magic, version, control_mask = _CONTROL_PACKET.unpack(packet)
            except struct.error:
                continue
            if (
                magic != _CONTROL_MAGIC
                or version != _CONTROL_VERSION
                or control_mask & ~_CONTROL_ALLOWED_MASK
            ):
                continue

            view_mask = int(
                control_mask & _CONTROL_VIEW_MASK
            )
            quest_yolo_requested = bool(
                control_mask & _CONTROL_YOLO_BIT
            )

            with self._condition:
                self._subscription_mask = view_mask
                self._quest_yolo_requested = (
                    quest_yolo_requested
                )
                self._last_subscription_time = time.monotonic()
                changed = (
                    self._last_logged_mask != view_mask
                    or self._last_logged_yolo
                    != quest_yolo_requested
                )
                self._last_logged_mask = view_mask
                self._last_logged_yolo = (
                    quest_yolo_requested
                )

            if changed:
                names = [
                    name
                    for name in self._view_names
                    if view_mask & (1 << VIEW_IDS[name])
                ]
                self._logger.info(
                    "[G1 Quest UDP] subscription mask=0x%04x "
                    "views=%s yolo=%s source=%s",
                    view_mask,
                    ",".join(names) if names else "none",
                    "on" if quest_yolo_requested else "off",
                    remote[0],
                )

    def _worker(self) -> None:
        while True:
            with self._condition:
                while not self._pending_frames and not self._stopping:
                    self._condition.wait()

                if self._stopping:
                    return

                frames = self._pending_frames
                self._pending_frames = {}
            try:
                self._send_batch(frames)
            except Exception as exc:
                self._warn(f"sender worker error: {exc}")

    def _send_batch(self, frames: dict[str, np.ndarray]) -> None:
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        frame_id = self._frame_id
        for name in self._view_names:
            frame = frames.get(name)
            if frame is not None:
                self._send_frame(name, frame, frame_id)

        now = time.monotonic()
        if now < self._next_stats_time:
            return
        with self._condition:
            replaced = self._replaced_frames
            self._replaced_frames = 0
        counts = ",".join(
            f"{name}:{self._frames_sent[name]}"
            for name in self._view_names if self._frames_sent[name]
        ) or "none"
        sizes = ",".join(
            f"{name}:{size / 1024.0:.1f}KiB/{chunks}"
            for name, (size, chunks) in self._last_sizes.items()
        ) or "none"
        self._logger.info(
            "[G1 Quest UDP] sent={%s} latest={%s} "
            "replaced_pending=%d target=%s:%d",
            counts, sizes, replaced, self._target[0], self._target[1],
        )
        self._next_stats_time = now + 5.0

    def _send_frame(self, name: str, frame: np.ndarray, frame_id: int) -> None:
        ok, encoded = cv2.imencode(
            ".jpg", frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not ok:
            self._warn(f"OpenCV JPEG encoding failed for {name}")
            return
        jpeg = encoded.tobytes()
        jpeg_size = len(jpeg)
        if jpeg_size == 0 or jpeg_size > self._max_jpeg_bytes:
            self._warn(
                f"discarding {name} JPEG of {jpeg_size} bytes; "
                f"limit is {self._max_jpeg_bytes}"
            )
            return
        chunk_count = (
            jpeg_size + self._payload_bytes - 1
        ) // self._payload_bytes
        if chunk_count > 0xFFFF:
            self._warn(f"{name} JPEG requires too many chunks: {chunk_count}")
            return

        view_id = VIEW_IDS[name]
        for chunk_index in range(chunk_count):
            start = chunk_index * self._payload_bytes
            payload = jpeg[start:start + self._payload_bytes]
            final_flag = 1 if chunk_index == chunk_count - 1 else 0
            flags = final_flag | (view_id << 1)
            header = _PACKET_HEADER.pack(
                _PACKET_MAGIC, _PACKET_VERSION, flags, frame_id,
                chunk_index, chunk_count, len(payload), jpeg_size,
            )
            try:
                self._socket.sendto(header + payload, self._target)
            except OSError as exc:
                self._warn(f"UDP send failed for {name}: {exc}")
                return
        self._frames_sent[name] += 1
        self._last_sizes[name] = (jpeg_size, chunk_count)

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if now >= self._next_warning_time:
            self._logger.warning("[G1 Quest UDP] %s", message)
            self._next_warning_time = now + 2.0


def create_quest_udp_sender(
    logger,
    stream_name: str = "realsense",
) -> QuestUdpJpegSender | None:
    if not _env_enabled("G1_QUEST_UDP_ENABLED", True):
        logger.info(
            "[G1 Quest UDP] disabled by G1_QUEST_UDP_ENABLED"
        )
        return None

    stream = str(stream_name).strip().lower()

    if stream == "realsense":
        view_names = tuple(
            name
            for name in VIEW_NAMES
            if name not in {"lifecam", "near", "topdown"}
        )
        fallback_mask = 1 << VIEW_IDS["rgb"]
        control_environment = (
            "G1_QUEST_UDP_CONTROL_PORT"
        )
        control_default = 5057
    elif stream == "lifecam":
        view_names = ("lifecam",)
        fallback_mask = 0
        control_environment = (
            "G1_QUEST_LIFECAM_CONTROL_PORT"
        )
        control_default = 5058
    else:
        logger.warning(
            "[G1 Quest UDP] unknown stream %r",
            stream_name,
        )
        return None

    try:
        host = os.environ.get(
            "G1_QUEST_UDP_HOST",
            "192.168.0.183",
        ).strip()

        if not host:
            raise ValueError(
                "G1_QUEST_UDP_HOST is empty"
            )

        return QuestUdpJpegSender(
            logger=logger,
            host=host,
            port=_env_int(
                "G1_QUEST_UDP_PORT",
                5056,
                1,
                65535,
            ),
            control_port=_env_int(
                control_environment,
                control_default,
                1,
                65535,
            ),
            subscription_timeout=_env_float(
                "G1_QUEST_UDP_SUBSCRIPTION_TIMEOUT",
                2.0,
                0.5,
                30.0,
            ),
            fps=_env_int(
                "G1_QUEST_UDP_FPS",
                30,
                1,
                30,
            ),
            jpeg_quality=_env_int(
                "G1_QUEST_UDP_JPEG_QUALITY",
                70,
                30,
                95,
            ),
            packet_bytes=_env_int(
                "G1_QUEST_UDP_PACKET_BYTES",
                1200,
                512,
                1400,
            ),
            max_jpeg_bytes=_env_int(
                "G1_QUEST_UDP_MAX_JPEG_BYTES",
                2_097_152,
                65_536,
                4_194_304,
            ),
            view_names=view_names,
            fallback_mask=fallback_mask,
        )
    except Exception as exc:
        logger.warning(
            "[G1 Quest UDP] %s sender disabled "
            "after setup failure: %s",
            stream,
            exc,
        )
        return None
