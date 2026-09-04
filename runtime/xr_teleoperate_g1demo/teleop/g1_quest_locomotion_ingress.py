#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import hmac
import json
import math
import os
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Any


MAGIC = b"G1L1"
VERSION = 1

BODY = struct.Struct("<4sHHQQQ5f")
HMAC_BYTES = 32
PACKET_BYTES = BODY.size + HMAC_BYTES

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 5059
DEFAULT_ALLOWED_IP = "192.168.0.183"
DEFAULT_STALE_TIMEOUT_S = 0.20


class QuestLocomotionIngress:
    def __init__(
        self,
        key: bytes,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        allowed_ip: str = DEFAULT_ALLOWED_IP,
        stale_timeout_s: float =
            DEFAULT_STALE_TIMEOUT_S,
    ) -> None:
        if len(key) < 32:
            raise ValueError(
                "Locomotion key must contain at least 32 bytes"
            )

        if not 0.10 <= stale_timeout_s <= 1.0:
            raise ValueError(
                "Stale timeout must be between 0.10 and 1.0 seconds"
            )

        self._key = bytes(key)
        self._host = str(host)
        self._port = int(port)
        self._allowed_ip = str(allowed_ip).strip()
        self._stale_timeout_s = float(stale_timeout_s)

        self._lock = threading.Lock()
        self._stop = threading.Event()

        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None

        self._session = 0
        self._sequence = 0
        self._received_at = 0.0

        self._left_x = 0.0
        self._left_y = 0.0
        self._right_x = 0.0
        self._right_y = 0.0
        self._deadman = 0.0

        self._source_ip: str | None = None
        self._packet_count = 0
        self._invalid_count = 0
        self._last_error: str | None = None

    @classmethod
    def from_environment(
        cls,
    ) -> "QuestLocomotionIngress":
        key_path = Path(
            os.environ.get(
                "G1_QUEST_LOCOMOTION_KEY_FILE",
                str(
                    Path.home()
                    / ".config/g1_dashboard/"
                    "unity_televuer.key"
                ),
            )
        ).expanduser()

        key_text = key_path.read_text(
            encoding="utf-8"
        ).strip()

        try:
            key = bytes.fromhex(key_text)
        except ValueError as exception:
            raise ValueError(
                f"Invalid hexadecimal key: {key_path}"
            ) from exception

        return cls(
            key=key,
            host=os.environ.get(
                "G1_QUEST_LOCOMOTION_HOST",
                DEFAULT_HOST,
            ),
            port=int(
                os.environ.get(
                    "G1_QUEST_LOCOMOTION_PORT",
                    str(DEFAULT_PORT),
                )
            ),
            allowed_ip=os.environ.get(
                "G1_QUEST_LOCOMOTION_ALLOWED_IP",
                DEFAULT_ALLOWED_IP,
            ),
            stale_timeout_s=float(
                os.environ.get(
                    "G1_QUEST_LOCOMOTION_STALE_S",
                    str(DEFAULT_STALE_TIMEOUT_S),
                )
            ),
        )

    def start(self) -> None:
        if self._thread is not None:
            return

        receiver = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        receiver.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        receiver.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_RCVBUF,
            256 * 1024,
        )

        receiver.bind(
            (self._host, self._port)
        )

        receiver.settimeout(0.10)

        self._socket = receiver
        self._stop.clear()

        self._thread = threading.Thread(
            target=self._receive_loop,
            name="g1-quest-locomotion-ingress",
            daemon=True,
        )

        self._thread.start()

    def close(self) -> None:
        self._stop.set()

        receiver = self._socket
        self._socket = None

        if receiver is not None:
            try:
                receiver.close()
            except OSError:
                pass

        thread = self._thread
        self._thread = None

        if (
            thread is not None
            and thread.is_alive()
        ):
            thread.join(timeout=1.0)

        self._clear_control()

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()

        with self._lock:
            age_s = (
                now - self._received_at
                if self._received_at > 0.0
                else math.inf
            )

            valid = (
                self._received_at > 0.0
                and age_s <= self._stale_timeout_s
            )

            deadman = (
                self._deadman
                if valid
                else 0.0
            )

            return {
                "valid": valid,
                "age_s": (
                    age_s
                    if math.isfinite(age_s)
                    else None
                ),
                "armed": (
                    valid and
                    deadman >= 0.75
                ),
                "left": (
                    [self._left_x, self._left_y]
                    if valid
                    else [0.0, 0.0]
                ),
                "right": (
                    [self._right_x, self._right_y]
                    if valid
                    else [0.0, 0.0]
                ),
                "deadman": deadman,
                "source_ip": self._source_ip,
                "session": self._session,
                "sequence": self._sequence,
                "packet_count": self._packet_count,
                "invalid_count": self._invalid_count,
                "last_error": self._last_error,
            }

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            receiver = self._socket

            if receiver is None:
                return

            try:
                packet, address = receiver.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue

            try:
                self._accept_packet(
                    packet,
                    str(address[0]),
                )
            except Exception as exception:
                with self._lock:
                    self._invalid_count += 1
                    self._last_error = str(exception)

    def _accept_packet(
        self,
        packet: bytes,
        source_ip: str,
    ) -> None:
        if (
            self._allowed_ip
            and source_ip != self._allowed_ip
        ):
            raise ValueError(
                f"source IP {source_ip} is not allowed"
            )

        if len(packet) != PACKET_BYTES:
            raise ValueError(
                f"packet size {len(packet)} != "
                f"{PACKET_BYTES}"
            )

        body = packet[:BODY.size]
        received_mac = packet[BODY.size:]

        expected_mac = hmac.new(
            self._key,
            body,
            hashlib.sha256,
        ).digest()

        if not hmac.compare_digest(
            expected_mac,
            received_mac,
        ):
            raise ValueError(
                "packet HMAC is invalid"
            )

        (
            magic,
            version,
            flags,
            session,
            sequence,
            client_ns,
            left_x,
            left_y,
            right_x,
            right_y,
            deadman,
        ) = BODY.unpack(body)

        if magic != MAGIC:
            raise ValueError("invalid magic")

        if version != VERSION:
            raise ValueError(
                f"unsupported version {version}"
            )

        if flags != 1:
            raise ValueError(
                f"invalid flags 0x{flags:x}"
            )

        if (
            session <= 0
            or sequence <= 0
            or client_ns <= 0
        ):
            raise ValueError(
                "invalid session, sequence, or timestamp"
            )

        values = (
            left_x,
            left_y,
            right_x,
            right_y,
            deadman,
        )

        if not all(
            math.isfinite(value)
            for value in values
        ):
            raise ValueError(
                "packet contains non-finite values"
            )

        if max(
            abs(left_x),
            abs(left_y),
            abs(right_x),
            abs(right_y),
        ) > 1.05:
            raise ValueError(
                "joystick value is outside [-1, 1]"
            )

        if not 0.0 <= deadman <= 1.01:
            raise ValueError(
                "deadman value is outside [0, 1]"
            )

        now = time.monotonic()

        with self._lock:
            if session != self._session:
                previous_fresh = (
                    self._received_at > 0.0
                    and now - self._received_at <= 0.50
                )

                if previous_fresh:
                    raise ValueError(
                        "another sender session is active"
                    )

                self._session = session
                self._sequence = 0

            if sequence <= self._sequence:
                raise ValueError(
                    "sequence did not increase"
                )

            self._sequence = sequence
            self._received_at = now

            self._left_x = float(left_x)
            self._left_y = float(left_y)
            self._right_x = float(right_x)
            self._right_y = float(right_y)
            self._deadman = float(deadman)

            self._source_ip = source_ip
            self._packet_count += 1
            self._last_error = None

    def _clear_control(self) -> None:
        with self._lock:
            self._received_at = 0.0
            self._left_x = 0.0
            self._left_y = 0.0
            self._right_x = 0.0
            self._right_y = 0.0
            self._deadman = 0.0


def main() -> int:
    ingress = QuestLocomotionIngress.from_environment()

    ingress.start()

    print(
        "Quest locomotion diagnostic receiver listening "
        f"on UDP {DEFAULT_PORT}. No robot commands are enabled.",
        flush=True,
    )

    try:
        while True:
            print(
                json.dumps(
                    ingress.snapshot(),
                    separators=(",", ":"),
                ),
                flush=True,
            )

            time.sleep(0.25)
    except KeyboardInterrupt:
        print("Stopping.", flush=True)
    finally:
        ingress.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())