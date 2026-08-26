#!/usr/bin/env python3
"""Latest-frame-only, MTU-safe JPEG-over-UDP sender for the Quest."""

from __future__ import annotations

import atexit
import os
import socket
import struct
import threading
import time

import cv2
import numpy as np


# Little-endian, 20-byte packet header:
#   magic         4 bytes: b"G1JP"
#   version       uint8:   1
#   flags         uint8:   bit 0 means final chunk
#   frame_id      uint32
#   chunk_index   uint16
#   chunk_count   uint16
#   payload_size  uint16
#   jpeg_size     uint32
_PACKET_HEADER = struct.Struct("<4sBBIHHHI")
_PACKET_MAGIC = b"G1JP"
_PACKET_VERSION = 1


def _env_enabled(name: str, default: bool) -> bool:
    raw = os.environ.get(name, "1" if default else "0").strip().lower()
    return raw not in {"0", "false", "no", "off", "disabled"}


def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
    value = int(os.environ.get(name, str(default)))
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


class QuestUdpJpegSender:
    """Encode and send only the newest submitted frame.

    JPEG encoding and UDP transmission happen on a daemon thread. The RealSense
    capture thread only rate-limits and copies the latest completed display
    frame, so a slow or disconnected Quest cannot stall camera acquisition.
    """

    def __init__(
        self,
        logger,
        host: str,
        port: int,
        fps: int,
        jpeg_quality: int,
        packet_bytes: int,
        max_jpeg_bytes: int,
    ):
        if packet_bytes <= _PACKET_HEADER.size:
            raise ValueError("UDP packet size is smaller than its header")

        address = socket.getaddrinfo(
            host,
            port,
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )[0][4]

        self._logger = logger
        self._target = address
        self._fps = fps
        self._jpeg_quality = jpeg_quality
        self._payload_bytes = packet_bytes - _PACKET_HEADER.size
        self._max_jpeg_bytes = max_jpeg_bytes
        self._frame_period = 1.0 / float(fps)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1_048_576)
        except OSError:
            pass

        self._condition = threading.Condition()
        self._pending_frame: np.ndarray | None = None
        self._stopping = False
        self._next_accept_time = 0.0
        self._frame_id = int(time.time_ns()) & 0xFFFFFFFF
        self._replaced_frames = 0
        self._frames_sent = 0
        self._next_stats_time = time.monotonic() + 5.0
        self._next_warning_time = 0.0

        self._thread = threading.Thread(
            target=self._worker,
            name="g1-quest-udp",
            daemon=True,
        )
        self._thread.start()
        atexit.register(self.close)

        self._logger.info(
            "[G1 Quest UDP] enabled target=%s:%d fps=%d quality=%d "
            "packet_bytes=%d header_bytes=%d",
            self._target[0],
            self._target[1],
            self._fps,
            self._jpeg_quality,
            packet_bytes,
            _PACKET_HEADER.size,
        )

    def submit(self, bgr: np.ndarray) -> None:
        """Offer a frame without allowing latency to accumulate."""
        try:
            now = time.monotonic()
            with self._condition:
                if self._stopping or now < self._next_accept_time:
                    return

                self._next_accept_time = now + self._frame_period
                frame_copy = np.array(bgr, dtype=np.uint8, copy=True, order="C")

                if self._pending_frame is not None:
                    self._replaced_frames += 1

                self._pending_frame = frame_copy
                self._condition.notify()
        except Exception as exc:
            self._warn(f"frame submission failed: {exc}")

    def close(self) -> None:
        with self._condition:
            if self._stopping:
                return
            self._stopping = True
            self._pending_frame = None
            self._condition.notify_all()

        try:
            self._socket.close()
        except OSError:
            pass

    def _worker(self) -> None:
        while True:
            with self._condition:
                while self._pending_frame is None and not self._stopping:
                    self._condition.wait()

                if self._stopping:
                    return

                frame = self._pending_frame
                self._pending_frame = None

            try:
                self._send_frame(frame)
            except Exception as exc:
                # The optional Quest stream must never kill the camera runner.
                self._warn(f"sender worker error: {exc}")

    def _send_frame(self, frame: np.ndarray) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
        )
        if not ok:
            self._warn("OpenCV JPEG encoding failed")
            return

        jpeg = encoded.tobytes()
        jpeg_size = len(jpeg)

        if jpeg_size == 0 or jpeg_size > self._max_jpeg_bytes:
            self._warn(
                f"discarding JPEG of {jpeg_size} bytes; "
                f"limit is {self._max_jpeg_bytes}"
            )
            return

        chunk_count = (
            jpeg_size + self._payload_bytes - 1
        ) // self._payload_bytes

        if chunk_count > 0xFFFF:
            self._warn(f"JPEG requires too many UDP chunks: {chunk_count}")
            return

        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        frame_id = self._frame_id

        for chunk_index in range(chunk_count):
            start = chunk_index * self._payload_bytes
            payload = jpeg[start : start + self._payload_bytes]
            flags = 1 if chunk_index == chunk_count - 1 else 0

            header = _PACKET_HEADER.pack(
                _PACKET_MAGIC,
                _PACKET_VERSION,
                flags,
                frame_id,
                chunk_index,
                chunk_count,
                len(payload),
                jpeg_size,
            )

            try:
                self._socket.sendto(header + payload, self._target)
            except OSError as exc:
                # This leaves an incomplete frame, which the receiver discards.
                self._warn(f"UDP send failed: {exc}")
                return

        self._frames_sent += 1
        now = time.monotonic()

        if now >= self._next_stats_time:
            with self._condition:
                replaced = self._replaced_frames
                self._replaced_frames = 0

            self._logger.info(
                "[G1 Quest UDP] sent_frames=%d latest_frame=%u "
                "jpeg_kib=%.1f chunks=%d replaced_pending=%d target=%s:%d",
                self._frames_sent,
                frame_id,
                jpeg_size / 1024.0,
                chunk_count,
                replaced,
                self._target[0],
                self._target[1],
            )
            self._next_stats_time = now + 5.0

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if now >= self._next_warning_time:
            self._logger.warning("[G1 Quest UDP] %s", message)
            self._next_warning_time = now + 2.0


def create_quest_udp_sender(logger) -> QuestUdpJpegSender | None:
    """Build the optional sender from environment variables.

    Defaults:
      G1_QUEST_UDP_ENABLED=1
      G1_QUEST_UDP_HOST=192.168.0.183
      G1_QUEST_UDP_PORT=5056
      G1_QUEST_UDP_FPS=20
      G1_QUEST_UDP_JPEG_QUALITY=70
      G1_QUEST_UDP_PACKET_BYTES=1200
      G1_QUEST_UDP_MAX_JPEG_BYTES=2097152
    """
    if not _env_enabled("G1_QUEST_UDP_ENABLED", True):
        logger.info("[G1 Quest UDP] disabled by G1_QUEST_UDP_ENABLED")
        return None

    try:
        host = os.environ.get(
            "G1_QUEST_UDP_HOST",
            "192.168.0.183",
        ).strip()
        if not host:
            raise ValueError("G1_QUEST_UDP_HOST is empty")

        return QuestUdpJpegSender(
            logger=logger,
            host=host,
            port=_env_int("G1_QUEST_UDP_PORT", 5056, 1, 65535),
            fps=_env_int("G1_QUEST_UDP_FPS", 20, 1, 30),
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
        )
    except Exception as exc:
        logger.warning("[G1 Quest UDP] disabled after setup failure: %s", exc)
        return None
