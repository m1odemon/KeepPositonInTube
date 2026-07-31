"""TI MSPM0G3507 21-byte vision protocol V1."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass


HEADER = b"\xA5\x5A"
PROTOCOL_VERSION = 1
FRAME_SIZE = 21
PAYLOAD_FORMAT = "<BIIff"


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_vision_frame(
    sequence: int,
    capture_timestamp_us: int,
    position_mm: float,
    confidence: float,
) -> bytes:
    if not math.isfinite(position_mm):
        raise ValueError("position_mm must be finite")
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be finite and in [0, 1]")
    payload = struct.pack(
        PAYLOAD_FORMAT,
        PROTOCOL_VERSION,
        sequence & 0xFFFFFFFF,
        capture_timestamp_us & 0xFFFFFFFF,
        float(position_mm),
        float(confidence),
    )
    crc = crc16_ccitt_false(payload)
    frame = HEADER + payload + struct.pack("<H", crc)
    if len(frame) != FRAME_SIZE:
        raise AssertionError("internal protocol frame-size error")
    return frame


@dataclass(frozen=True, slots=True)
class DecodedVisionFrame:
    sequence: int
    capture_timestamp_us: int
    position_mm: float
    confidence: float


def decode_vision_frame(frame: bytes) -> DecodedVisionFrame:
    if len(frame) != FRAME_SIZE:
        raise ValueError("vision frame must be exactly 21 bytes")
    if frame[:2] != HEADER:
        raise ValueError("invalid frame header")
    payload = frame[2:19]
    expected_crc = struct.unpack("<H", frame[19:21])[0]
    if crc16_ccitt_false(payload) != expected_crc:
        raise ValueError("CRC mismatch")
    version, sequence, timestamp, position, confidence = struct.unpack(
        PAYLOAD_FORMAT,
        payload,
    )
    if version != PROTOCOL_VERSION:
        raise ValueError("unsupported protocol version")
    if not math.isfinite(position):
        raise ValueError("non-finite position")
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("invalid confidence")
    return DecodedVisionFrame(
        sequence,
        timestamp,
        position,
        confidence,
    )
