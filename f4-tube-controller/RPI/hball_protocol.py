"""H题滚球系统树莓派/F407二进制协议（版本1）。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Generic, TypeVar


PROTOCOL_VERSION = 1
PI_COMMAND_TYPE = 0x10
TELEMETRY_TYPE = 0x90
PI_COMMAND_HEADER = b"\xA5\x5A"
TELEMETRY_HEADER = b"\x5A\xA5"

COMMAND_RUN = 1 << 0
COMMAND_EMERGENCY_STOP = 1 << 1
COMMAND_TARGET_VALID = 1 << 2
COMMAND_ACCELERATION_VALID = 1 << 3

_COMMAND_STRUCT = struct.Struct("<2sBBIIffffBBH")
_TELEMETRY_STRUCT = struct.Struct("<2sBBII8fHHBHBH")

PI_COMMAND_FRAME_SIZE = _COMMAND_STRUCT.size
TELEMETRY_FRAME_SIZE = _TELEMETRY_STRUCT.size


def crc16_ccitt_false(data: bytes | bytearray | memoryview) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def is_sequence_newer(candidate: int, reference: int) -> bool:
    delta = ((candidate - reference) & 0xFFFFFFFF)
    return 0 < delta < 0x80000000


@dataclass(frozen=True)
class PiCommand:
    sequence: int
    capture_timestamp_us: int
    ball_position_mm: float
    confidence: float
    target_position_mm: float
    chassis_acceleration_mps2: float = 0.0
    task_id: int = 0
    flags: int = 0


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    uptime_ms: int
    ball_position_mm: float
    ball_velocity_mm_s: float
    target_position_mm: float
    target_tube_angle_rad: float
    actual_tube_angle_rad: float
    motor_angle_rad: float
    motor_speed_rpm: float
    motor_current_a: float
    vision_age_ms: int
    motor_feedback_age_ms: int
    state: int
    fault_flags: int
    flags: int


def encode_pi_command(command: PiCommand) -> bytes:
    values = (
        command.ball_position_mm,
        command.confidence,
        command.target_position_mm,
        command.chassis_acceleration_mps2,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("command contains NaN or infinity")
    if not 0.0 <= command.confidence <= 1.0:
        raise ValueError("confidence must be in [0, 1]")

    without_crc = _COMMAND_STRUCT.pack(
        PI_COMMAND_HEADER,
        PROTOCOL_VERSION,
        PI_COMMAND_TYPE,
        command.sequence & 0xFFFFFFFF,
        command.capture_timestamp_us & 0xFFFFFFFF,
        *values,
        command.task_id & 0xFF,
        command.flags & 0xFF,
        0,
    )
    crc = crc16_ccitt_false(without_crc[2:-2])
    return without_crc[:-2] + struct.pack("<H", crc)


def decode_pi_command(frame: bytes) -> PiCommand:
    if len(frame) != PI_COMMAND_FRAME_SIZE:
        raise ValueError("incorrect command frame length")
    unpacked = _COMMAND_STRUCT.unpack(frame)
    if unpacked[0] != PI_COMMAND_HEADER:
        raise ValueError("incorrect command header")
    if unpacked[1] != PROTOCOL_VERSION or unpacked[2] != PI_COMMAND_TYPE:
        raise ValueError("unsupported command version or type")
    if crc16_ccitt_false(frame[2:-2]) != unpacked[-1]:
        raise ValueError("command CRC mismatch")
    return PiCommand(
        sequence=unpacked[3],
        capture_timestamp_us=unpacked[4],
        ball_position_mm=unpacked[5],
        confidence=unpacked[6],
        target_position_mm=unpacked[7],
        chassis_acceleration_mps2=unpacked[8],
        task_id=unpacked[9],
        flags=unpacked[10],
    )


def decode_telemetry(frame: bytes) -> Telemetry:
    if len(frame) != TELEMETRY_FRAME_SIZE:
        raise ValueError("incorrect telemetry frame length")
    unpacked = _TELEMETRY_STRUCT.unpack(frame)
    if unpacked[0] != TELEMETRY_HEADER:
        raise ValueError("incorrect telemetry header")
    if unpacked[1] != PROTOCOL_VERSION or unpacked[2] != TELEMETRY_TYPE:
        raise ValueError("unsupported telemetry version or type")
    if crc16_ccitt_false(frame[2:-2]) != unpacked[-1]:
        raise ValueError("telemetry CRC mismatch")

    return Telemetry(
        sequence=unpacked[3],
        uptime_ms=unpacked[4],
        ball_position_mm=unpacked[5],
        ball_velocity_mm_s=unpacked[6],
        target_position_mm=unpacked[7],
        target_tube_angle_rad=unpacked[8],
        actual_tube_angle_rad=unpacked[9],
        motor_angle_rad=unpacked[10],
        motor_speed_rpm=unpacked[11],
        motor_current_a=unpacked[12],
        vision_age_ms=unpacked[13],
        motor_feedback_age_ms=unpacked[14],
        state=unpacked[15],
        fault_flags=unpacked[16],
        flags=unpacked[17],
    )


T = TypeVar("T")


class FixedFrameDecoder(Generic[T]):
    """带帧头搜索、CRC失败重同步的固定长度流解析器。"""

    def __init__(self, header: bytes, frame_size: int, decoder):
        self._header = header
        self._frame_size = frame_size
        self._decoder = decoder
        self._buffer = bytearray()
        self.rejected_frames = 0

    def feed(self, data: bytes) -> list[T]:
        self._buffer.extend(data)
        decoded: list[T] = []
        while True:
            start = self._buffer.find(self._header)
            if start < 0:
                self._buffer[:] = self._buffer[-1:]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < self._frame_size:
                break

            candidate = bytes(self._buffer[: self._frame_size])
            try:
                decoded.append(self._decoder(candidate))
                del self._buffer[: self._frame_size]
            except ValueError:
                self.rejected_frames += 1
                del self._buffer[0]
        return decoded


def make_telemetry_decoder() -> FixedFrameDecoder[Telemetry]:
    return FixedFrameDecoder(
        TELEMETRY_HEADER, TELEMETRY_FRAME_SIZE, decode_telemetry
    )


assert PI_COMMAND_FRAME_SIZE == 32
assert TELEMETRY_FRAME_SIZE == 54

