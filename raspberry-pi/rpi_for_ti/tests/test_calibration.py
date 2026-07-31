from __future__ import annotations

import sys
import unittest
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))


class CalibrationTests(unittest.TestCase):
    def test_piecewise_position_mapping(self) -> None:
        from hball_ti.calibration import PositionCalibration

        calibration = PositionCalibration.from_dict(
            {
                "position_min_mm": -90,
                "position_max_mm": 110,
                "zero_u": 0.45,
                "camera_matrix": None,
                "distortion_coeffs": None,
            }
        )
        self.assertAlmostEqual(calibration.position_from_u(0.0), -90.0)
        self.assertAlmostEqual(calibration.position_from_u(0.45), 0.0)
        self.assertAlmostEqual(calibration.position_from_u(1.0), 110.0)
        self.assertAlmostEqual(calibration.u_from_position(-90.0), 0.0)
        self.assertAlmostEqual(calibration.u_from_position(0.0), 0.45)
        self.assertAlmostEqual(calibration.u_from_position(110.0), 1.0)


if __name__ == "__main__":
    unittest.main()
