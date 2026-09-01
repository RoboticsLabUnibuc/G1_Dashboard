#!/usr/bin/env python3
"""Authenticated Unity-to-TeleVuer WebSocket ingress."""

from __future__ import annotations

import hashlib
import hmac
import json
import logging
import os
import secrets
import struct
import threading
import time
from pathlib import Path
from typing import Any

from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve


LOG = logging.getLogger(
    "g1_locomotion_xr_arms_fingers_v5"
)

PROTOCOL_VERSION = 1
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8013
DEFAULT_ALLOWED_IP = "192.168.0.183"
MAX_CLOCK_ERROR_MS = 15_000
NONCE_LIFETIME_S = 30.0

OBSERVATION_MAGIC = b"G1F1"
OBSERVATION_MATRIX_COUNT = 51
OBSERVATION_STATE_FLOATS = 4
OBSERVATION_FLOATS = (
    OBSERVATION_MATRIX_COUNT * 16
    + OBSERVATION_STATE_FLOATS
)
OBSERVATION_HEADER = struct.Struct("<4sQQI")
OBSERVATION_HMAC_BYTES = 32
OBSERVATION_FRAME_BYTES = (
    OBSERVATION_HEADER.size
    + OBSERVATION_FLOATS * 4
    + OBSERVATION_HMAC_BYTES
)
OBSERVATION_REQUIRED_FLAGS = 0x07

DEFAULT_MOTION_ENABLED = False
DEFAULT_STABLE_FRAMES = 12
DEFAULT_STALE_TIMEOUT_S = 0.25
PINCH_THRESHOLD_M = 0.03
SQUEEZE_THRESHOLD = 0.65


def _environment_flag(name: str, default: bool) -> bool:
    value = os.environ.get(name)

    if value is None:
        return bool(default)

    normalized = value.strip().lower()

    if normalized in {"1", "true", "yes", "on", "enabled"}:
        return True

    if normalized in {"0", "false", "no", "off", "disabled"}:
        return False

    raise ValueError(f"{name} has invalid boolean value {value!r}")


class UnityTeleVuerIngress:
    """Authenticated side-channel feeding an existing TeleVuer instance.

    This first revision only authenticates the Unity client. Motion frames are
    deliberately rejected until transport verification is complete.
    """

    def __init__(
        self,
        tvuer: Any,
        key: bytes,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        allowed_ip: str = DEFAULT_ALLOWED_IP,
        motion_enabled: bool = DEFAULT_MOTION_ENABLED,
        stable_frames: int = DEFAULT_STABLE_FRAMES,
        stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
    ) -> None:
        if len(key) < 32:
            raise ValueError("Unity TeleVuer key must contain at least 32 bytes")

        self._tvuer = tvuer
        self._key = bytes(key)
        self._host = str(host)
        self._port = int(port)
        self._allowed_ip = str(allowed_ip).strip()
        self._motion_enabled = bool(motion_enabled)
        self._stable_frames = int(stable_frames)
        self._stale_timeout_s = float(stale_timeout_s)

        if self._stable_frames < 3:
            raise ValueError("Unity motion stable-frame count must be at least 3")

        if not 0.10 <= self._stale_timeout_s <= 1.0:
            raise ValueError(
                "Unity motion stale timeout must be between 0.10 and 1.0 seconds"
            )

        self._server = None
        self._thread: threading.Thread | None = None
        self._nonce_lock = threading.Lock()
        self._nonces: dict[str, float] = {}
        self._client_lock = threading.Lock()
        self._motion_source_claimed = False

    @classmethod
    def from_environment(cls, tvuer: Any) -> "UnityTeleVuerIngress":
        key_path = Path(
            os.environ.get(
                "G1_UNITY_TELEVUER_KEY_FILE",
                str(
                    Path.home()
                    / ".config/g1_dashboard/unity_televuer.key"
                ),
            )
        ).expanduser()

        key_text = key_path.read_text(encoding="utf-8").strip()

        try:
            key = bytes.fromhex(key_text)
        except ValueError as exc:
            raise ValueError(
                f"Invalid hexadecimal Unity TeleVuer key: {key_path}"
            ) from exc

        return cls(
            tvuer=tvuer,
            key=key,
            host=os.environ.get(
                "G1_UNITY_TELEVUER_HOST",
                DEFAULT_HOST,
            ),
            port=int(
                os.environ.get(
                    "G1_UNITY_TELEVUER_PORT",
                    str(DEFAULT_PORT),
                )
            ),
            allowed_ip=os.environ.get(
                "G1_UNITY_TELEVUER_ALLOWED_IP",
                DEFAULT_ALLOWED_IP,
            ),
            motion_enabled=_environment_flag(
                "G1_UNITY_TELEVUER_MOTION_ENABLED",
                DEFAULT_MOTION_ENABLED,
            ),
            stable_frames=int(
                os.environ.get(
                    "G1_UNITY_TELEVUER_STABLE_FRAMES",
                    str(DEFAULT_STABLE_FRAMES),
                )
            ),
            stale_timeout_s=float(
                os.environ.get(
                    "G1_UNITY_TELEVUER_STALE_TIMEOUT_S",
                    str(DEFAULT_STALE_TIMEOUT_S),
                )
            ),
        )

    def start(self) -> None:
        if self._server is not None:
            return

        self._server = serve(
            self._handle_connection,
            self._host,
            self._port,
            compression=None,
            max_size=4096,
            max_queue=4,
            ping_interval=10.0,
            ping_timeout=10.0,
            close_timeout=2.0,
        )

        if self._motion_enabled:
            required = (
                "activate_unity_motion_source",
                "release_unity_motion_source",
                "head_pose_shared",
                "left_arm_pose_shared",
                "right_arm_pose_shared",
                "left_hand_position_shared",
                "right_hand_position_shared",
                "left_hand_orientation_shared",
                "right_hand_orientation_shared",
                "motion_data_ready_shared",
            )

            missing = [
                name
                for name in required
                if not hasattr(self._tvuer, name)
            ]

            if missing:
                self._server.shutdown()
                self._server = None
                raise RuntimeError(
                    "TeleVuer does not expose Unity motion integration: "
                    + ", ".join(missing)
                )

            self._tvuer.activate_unity_motion_source()
            self._motion_source_claimed = True

        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="g1-unity-televuer-ingress",
            daemon=True,
        )
        self._thread.start()

        LOG.info(
            "[G1 Unity TeleVuer] authenticated ingress listening on "
            "ws://%s:%d allowed_ip=%s mode=%s stable_frames=%d stale=%.3fs",
            self._host,
            self._port,
            self._allowed_ip or "any",
            "motion" if self._motion_enabled else "observation",
            self._stable_frames,
            self._stale_timeout_s,
        )

    def close(self) -> None:
        self._mark_motion_unready()

        server = self._server
        self._server = None

        if server is not None:
            server.shutdown()

        thread = self._thread
        self._thread = None

        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

        if self._motion_source_claimed:
            self._tvuer.release_unity_motion_source()
            self._motion_source_claimed = False

        LOG.info("[G1 Unity TeleVuer] ingress stopped")

    def _handle_connection(self, connection: Any) -> None:
        if not self._client_lock.acquire(blocking=False):
            connection.close(1013, "Unity tracking client already connected")
            return

        try:
            self._handle_connection_exclusive(connection)
        finally:
            self._mark_motion_unready()
            self._client_lock.release()

    def _handle_connection_exclusive(self, connection: Any) -> None:
        remote = connection.remote_address
        remote_ip = str(remote[0]) if remote else ""

        if self._allowed_ip and remote_ip != self._allowed_ip:
            LOG.warning(
                "[G1 Unity TeleVuer] rejected remote address %s",
                remote_ip,
            )
            connection.close(1008, "remote address not allowed")
            return

        challenge_nonce = secrets.token_hex(32)

        connection.send(
            json.dumps(
                {
                    "type": "challenge",
                    "version": PROTOCOL_VERSION,
                    "nonce": challenge_nonce,
                    "server_unix_ms": int(time.time() * 1000),
                },
                separators=(",", ":"),
            )
        )

        try:
            message = connection.recv(timeout=5.0)
            request = self._decode_authentication(
                message,
                challenge_nonce,
            )
        except Exception as exc:
            LOG.warning(
                "[G1 Unity TeleVuer] authentication failed from %s: %s",
                remote_ip,
                exc,
            )
            connection.close(1008, "authentication failed")
            return

        session = secrets.token_hex(16)

        LOG.info(
            "[G1 Unity TeleVuer] authenticated client=%s nonce=%s",
            remote_ip,
            request["nonce"][:12],
        )

        connection.send(
            json.dumps(
                {
                    "type": "hello_ack",
                    "version": PROTOCOL_VERSION,
                    "server_unix_ms": int(time.time() * 1000),
                    "session": session,
                    "motion_enabled": self._motion_enabled,
                    "observation_enabled": True,
                },
                separators=(",", ":"),
            )
        )

        self._receive_observation_frames(
            connection,
            session,
            remote_ip,
        )

    def _receive_observation_frames(
        self,
        connection: Any,
        session: str,
        remote_ip: str,
    ) -> None:
        last_sequence = 0
        accepted = 0
        stable_frames = 0
        last_valid_frame = 0.0
        started = time.monotonic()
        last_report = started
        timeout_s = min(
            0.05,
            self._stale_timeout_s * 0.5,
        )

        while True:
            try:
                message = connection.recv(timeout=timeout_s)
            except TimeoutError:
                now = time.monotonic()

                if (
                    self._motion_enabled
                    and last_valid_frame > 0.0
                    and now - last_valid_frame
                    > self._stale_timeout_s
                ):
                    if self._mark_motion_unready():
                        LOG.warning(
                            "[G1 Unity TeleVuer] tracking stale after "
                            "%.3f s; motion_data_ready cleared",
                            now - last_valid_frame,
                        )

                    stable_frames = 0
                    last_valid_frame = 0.0

                continue
            except ConnectionClosed:
                break

            if isinstance(message, str):
                try:
                    value = json.loads(message)
                except json.JSONDecodeError:
                    value = {}

                if value.get("type") == "ping":
                    connection.send(
                        json.dumps(
                            {
                                "type": "pong",
                                "server_unix_ms": int(time.time() * 1000),
                            },
                            separators=(",", ":"),
                        )
                    )
                    continue

                connection.close(
                    1003,
                    "unexpected text message",
                )
                return

            try:
                frame = self._decode_observation_frame(
                    message,
                    session,
                    last_sequence,
                )
            except Exception as exc:
                LOG.warning(
                    "[G1 Unity TeleVuer] invalid observation "
                    "frame from %s: %s",
                    remote_ip,
                    exc,
                )
                connection.close(
                    1008,
                    "invalid observation frame",
                )
                return

            last_sequence = frame["sequence"]
            accepted += 1
            now = time.monotonic()
            last_valid_frame = now

            if self._motion_enabled:
                stable_frames = min(
                    stable_frames + 1,
                    self._stable_frames,
                )

                if stable_frames >= self._stable_frames:
                    became_ready = self._apply_motion_frame(frame)

                    if became_ready:
                        LOG.warning(
                            "[G1 Unity TeleVuer] Unity tracking qualified "
                            "after %d stable frames; TeleVuer input is ready "
                            "but robot ownership remains controlled by the "
                            "existing listener state machine",
                            self._stable_frames,
                        )

            if now - last_report >= 5.0:
                elapsed = max(now - started, 1e-6)
                rate = accepted / elapsed

                head = frame["matrices"][0]
                left_wrist = frame["matrices"][1]
                right_wrist = frame["matrices"][26]

                LOG.info(
                    "[G1 Unity TeleVuer] %s "
                    "client=%s accepted=%d rate=%.1fHz "
                    "sequence=%d head=(%.3f,%.3f,%.3f) "
                    "left=(%.3f,%.3f,%.3f) "
                    "right=(%.3f,%.3f,%.3f) "
                    "left_extent=%.3fm right_extent=%.3fm "
                    "stable=%d/%d",
                    (
                        "motion-input"
                        if self._motion_enabled
                        else "observation-only"
                    ),
                    remote_ip,
                    accepted,
                    rate,
                    last_sequence,
                    head[12],
                    head[13],
                    head[14],
                    left_wrist[12],
                    left_wrist[13],
                    left_wrist[14],
                    right_wrist[12],
                    right_wrist[13],
                    right_wrist[14],
                    frame["left_extent"],
                    frame["right_extent"],
                    stable_frames,
                    self._stable_frames,
                )
                last_report = now

        LOG.info(
            "[G1 Unity TeleVuer] tracking client disconnected: %s",
            remote_ip,
        )

    @staticmethod
    def _write_shared_array(shared: Any, values: list[float]) -> None:
        with shared.get_lock():
            shared[:] = values

    def _apply_motion_frame(
        self,
        frame: dict[str, Any],
    ) -> bool:
        matrices = frame["matrices"]
        left_hand = matrices[1:26]
        right_hand = matrices[26:51]

        left_positions = [
            value
            for matrix in left_hand
            for value in (
                matrix[12],
                matrix[13],
                matrix[14],
            )
        ]
        right_positions = [
            value
            for matrix in right_hand
            for value in (
                matrix[12],
                matrix[13],
                matrix[14],
            )
        ]

        orientation_indices = (
            0, 1, 2,
            4, 5, 6,
            8, 9, 10,
        )

        left_orientations = [
            matrix[index]
            for matrix in left_hand
            for index in orientation_indices
        ]
        right_orientations = [
            matrix[index]
            for matrix in right_hand
            for index in orientation_indices
        ]

        self._write_shared_array(
            self._tvuer.head_pose_shared,
            list(matrices[0]),
        )
        self._write_shared_array(
            self._tvuer.left_arm_pose_shared,
            list(left_hand[0]),
        )
        self._write_shared_array(
            self._tvuer.right_arm_pose_shared,
            list(right_hand[0]),
        )
        self._write_shared_array(
            self._tvuer.left_hand_position_shared,
            left_positions,
        )
        self._write_shared_array(
            self._tvuer.right_hand_position_shared,
            right_positions,
        )
        self._write_shared_array(
            self._tvuer.left_hand_orientation_shared,
            left_orientations,
        )
        self._write_shared_array(
            self._tvuer.right_hand_orientation_shared,
            right_orientations,
        )

        (
            left_pinch,
            left_squeeze,
            right_pinch,
            right_squeeze,
        ) = frame["states"]

        state_values = (
            (
                "left",
                float(left_pinch),
                float(left_squeeze),
            ),
            (
                "right",
                float(right_pinch),
                float(right_squeeze),
            ),
        )

        for side, pinch, squeeze in state_values:
            pinch_flag = getattr(
                self._tvuer,
                f"{side}_hand_pinch_shared",
            )
            pinch_value = getattr(
                self._tvuer,
                f"{side}_hand_pinchValue_shared",
            )
            squeeze_flag = getattr(
                self._tvuer,
                f"{side}_hand_squeeze_shared",
            )
            squeeze_value = getattr(
                self._tvuer,
                f"{side}_hand_squeezeValue_shared",
            )

            with pinch_flag.get_lock():
                pinch_flag.value = (
                    pinch <= PINCH_THRESHOLD_M
                )

            with pinch_value.get_lock():
                pinch_value.value = pinch

            with squeeze_flag.get_lock():
                squeeze_flag.value = (
                    squeeze >= SQUEEZE_THRESHOLD
                )

            with squeeze_value.get_lock():
                squeeze_value.value = squeeze

        with self._tvuer.motion_data_ready_shared.get_lock():
            was_ready = bool(
                self._tvuer.motion_data_ready_shared.value
            )
            self._tvuer.motion_data_ready_shared.value = True

        return not was_ready

    def _mark_motion_unready(self) -> bool:
        if not self._motion_source_claimed:
            return False

        with self._tvuer.motion_data_ready_shared.get_lock():
            was_ready = bool(
                self._tvuer.motion_data_ready_shared.value
            )
            self._tvuer.motion_data_ready_shared.value = False

        return was_ready

    def _decode_observation_frame(
        self,
        message: Any,
        session: str,
        last_sequence: int,
    ) -> dict[str, Any]:
        if not isinstance(message, bytes):
            raise ValueError("observation frame must be binary")

        if len(message) != OBSERVATION_FRAME_BYTES:
            raise ValueError(
                f"frame size {len(message)} != "
                f"{OBSERVATION_FRAME_BYTES}"
            )

        body = message[:-OBSERVATION_HMAC_BYTES]
        received_mac = message[-OBSERVATION_HMAC_BYTES:]

        expected_mac = hmac.new(
            self._key,
            session.encode("ascii") + body,
            hashlib.sha256,
        ).digest()

        if not hmac.compare_digest(
            expected_mac,
            received_mac,
        ):
            raise ValueError("frame HMAC is invalid")

        magic, sequence, client_ns, flags = (
            OBSERVATION_HEADER.unpack_from(body, 0)
        )

        if magic != OBSERVATION_MAGIC:
            raise ValueError("frame magic is invalid")

        if sequence <= last_sequence:
            raise ValueError("frame sequence is not increasing")

        if client_ns <= 0:
            raise ValueError("client monotonic time is invalid")

        if flags != OBSERVATION_REQUIRED_FLAGS:
            raise ValueError(
                f"tracking flags 0x{flags:x} are incomplete"
            )

        values = struct.unpack_from(
            f"<{OBSERVATION_FLOATS}f",
            body,
            OBSERVATION_HEADER.size,
        )

        matrices = [
            values[index:index + 16]
            for index in range(
                0,
                OBSERVATION_MATRIX_COUNT * 16,
                16,
            )
        ]

        for index, matrix in enumerate(matrices):
            self._validate_matrix(matrix, index)

        left_hand = matrices[1:26]
        right_hand = matrices[26:51]

        left_extent = self._validate_hand(
            left_hand,
            "left",
        )
        right_extent = self._validate_hand(
            right_hand,
            "right",
        )

        state_offset = OBSERVATION_MATRIX_COUNT * 16
        states = values[
            state_offset:
            state_offset + OBSERVATION_STATE_FLOATS
        ]

        left_pinch, left_squeeze, right_pinch, right_squeeze = states

        if not (
            0.0 <= left_pinch <= 0.5
            and 0.0 <= right_pinch <= 0.5
            and 0.0 <= left_squeeze <= 1.0
            and 0.0 <= right_squeeze <= 1.0
        ):
            raise ValueError("hand state values are invalid")

        return {
            "sequence": sequence,
            "client_ns": client_ns,
            "matrices": matrices,
            "left_extent": left_extent,
            "right_extent": right_extent,
            "states": states,
        }

    @staticmethod
    def _validate_matrix(
        matrix: tuple[float, ...],
        index: int,
    ) -> None:
        if not all(
            value == value
            and abs(value) != float("inf")
            for value in matrix
        ):
            raise ValueError(
                f"matrix {index} contains non-finite values"
            )

        if (
            abs(matrix[3]) > 1e-3
            or abs(matrix[7]) > 1e-3
            or abs(matrix[11]) > 1e-3
            or abs(matrix[15] - 1.0) > 1e-3
        ):
            raise ValueError(
                f"matrix {index} has an invalid homogeneous row"
            )

        if max(
            abs(matrix[12]),
            abs(matrix[13]),
            abs(matrix[14]),
        ) > 10.0:
            raise ValueError(
                f"matrix {index} position is out of range"
            )

        r00, r01, r02 = matrix[0], matrix[4], matrix[8]
        r10, r11, r12 = matrix[1], matrix[5], matrix[9]
        r20, r21, r22 = matrix[2], matrix[6], matrix[10]

        determinant = (
            r00 * (r11 * r22 - r12 * r21)
            - r01 * (r10 * r22 - r12 * r20)
            + r02 * (r10 * r21 - r11 * r20)
        )

        if not 0.7 <= determinant <= 1.3:
            raise ValueError(
                f"matrix {index} rotation determinant "
                f"is {determinant:.4f}"
            )

    @staticmethod
    def _validate_hand(
        matrices: list[tuple[float, ...]],
        side: str,
    ) -> float:
        wrist = matrices[0]
        wx, wy, wz = wrist[12], wrist[13], wrist[14]

        extent = max(
            (
                (
                    (matrix[12] - wx) ** 2
                    + (matrix[13] - wy) ** 2
                    + (matrix[14] - wz) ** 2
                )
                ** 0.5
            )
            for matrix in matrices
        )

        if not 0.03 <= extent <= 0.50:
            raise ValueError(
                f"{side} hand extent {extent:.4f} m "
                "is outside the valid range"
            )

        return extent

    def _decode_authentication(
        self,
        message: Any,
        challenge_nonce: str,
    ) -> dict[str, Any]:
        if not isinstance(message, str):
            raise ValueError(
                "authentication must be a text message"
            )

        request = json.loads(message)

        if request.get("type") != "authenticate":
            raise ValueError("unexpected message type")

        if int(request.get("version", 0)) != PROTOCOL_VERSION:
            raise ValueError("unsupported protocol version")

        nonce = str(request["nonce"]).lower()
        received_mac = str(request["mac"]).lower()

        if not hmac.compare_digest(nonce, challenge_nonce):
            raise ValueError("challenge nonce does not match")

        signed = (
            f"G1TV1|AUTH|{nonce}"
        ).encode("ascii")

        expected_mac = hmac.new(
            self._key,
            signed,
            hashlib.sha256,
        ).hexdigest()

        if not hmac.compare_digest(
            expected_mac,
            received_mac,
        ):
            raise ValueError(
                "invalid message authentication code"
            )

        return {
            "nonce": nonce,
        }
