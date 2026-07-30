from __future__ import annotations

from pathlib import Path
import struct
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "RPI"))

from hball_protocol import (  # noqa: E402
    COMMAND_RUN,
    COMMAND_TARGET_VALID,
    PI_COMMAND_FRAME_SIZE,
    TELEMETRY_FRAME_SIZE,
    PiCommand,
    crc16_ccitt_false,
    decode_pi_command,
    decode_telemetry,
    encode_pi_command,
    is_sequence_newer,
    make_telemetry_decoder,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_standard_vector(self) -> None:
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_command_round_trip(self) -> None:
        source = PiCommand(
            sequence=0x12345678,
            capture_timestamp_us=42_000,
            ball_position_mm=-12.5,
            confidence=0.875,
            target_position_mm=50.0,
            task_id=3,
            flags=COMMAND_RUN | COMMAND_TARGET_VALID,
        )
        frame = encode_pi_command(source)
        self.assertEqual(len(frame), PI_COMMAND_FRAME_SIZE)
        decoded = decode_pi_command(frame)
        self.assertEqual(decoded.sequence, source.sequence)
        self.assertAlmostEqual(decoded.ball_position_mm, -12.5)
        self.assertAlmostEqual(decoded.confidence, 0.875)
        self.assertAlmostEqual(decoded.target_position_mm, 50.0)
        self.assertEqual(decoded.flags, source.flags)

    def test_command_crc_rejects_corruption(self) -> None:
        frame = bytearray(
            encode_pi_command(
                PiCommand(
                    sequence=1,
                    capture_timestamp_us=2,
                    ball_position_mm=3.0,
                    confidence=1.0,
                    target_position_mm=0.0,
                )
            )
        )
        frame[12] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC"):
            decode_pi_command(bytes(frame))

    def test_sequence_wrap(self) -> None:
        self.assertTrue(is_sequence_newer(0, 0xFFFFFFFF))
        self.assertTrue(is_sequence_newer(100, 99))
        self.assertFalse(is_sequence_newer(99, 100))
        self.assertFalse(is_sequence_newer(7, 7))

    def test_telemetry_stream_resynchronizes(self) -> None:
        values = (
            b"\x5A\xA5",
            1,
            0x90,
            7,
            1234,
            *([0.0] * 8),
            65535,
            10,
            1,
            1,
            0,
            0,
        )
        raw = bytearray(struct.pack("<2sBBII8fHHBHBH", *values))
        crc = crc16_ccitt_false(raw[2:-2])
        raw[-2:] = struct.pack("<H", crc)
        self.assertEqual(len(raw), TELEMETRY_FRAME_SIZE)
        decoded = decode_telemetry(bytes(raw))
        self.assertEqual(decoded.state, 1)

        corrupt = bytearray(raw)
        corrupt[20] ^= 0x80
        stream = make_telemetry_decoder()
        frames = stream.feed(b"\x00\x11garbage" + corrupt + raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 7)
        self.assertGreaterEqual(stream.rejected_frames, 1)


if __name__ == "__main__":
    unittest.main()

