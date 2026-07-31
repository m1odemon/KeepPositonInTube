from __future__ import annotations

import sys
import unittest
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))


class ProtocolTests(unittest.TestCase):
    def test_standard_crc_vector(self) -> None:
        from hball_ti.ti_protocol import crc16_ccitt_false

        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_documented_21_byte_vector(self) -> None:
        from hball_ti.ti_protocol import make_vision_frame

        frame = make_vision_frame(1, 1_000_000, 0.0, 1.0)
        self.assertEqual(len(frame), 21)
        self.assertEqual(
            frame.hex(" ").upper(),
            (
                "A5 5A 01 01 00 00 00 40 42 0F 00 "
                "00 00 00 00 00 00 80 3F 99 CC"
            ),
        )

    def test_round_trip(self) -> None:
        from hball_ti.ti_protocol import (
            decode_vision_frame,
            make_vision_frame,
        )

        decoded = decode_vision_frame(
            make_vision_frame(0xFFFFFFFE, 123456, -42.5, 0.75)
        )
        self.assertEqual(decoded.sequence, 0xFFFFFFFE)
        self.assertEqual(decoded.capture_timestamp_us, 123456)
        self.assertAlmostEqual(decoded.position_mm, -42.5)
        self.assertAlmostEqual(decoded.confidence, 0.75)


if __name__ == "__main__":
    unittest.main()
