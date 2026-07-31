from __future__ import annotations

import sys
import unittest
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))


class TravelTimerTests(unittest.TestCase):
    def test_positive_direction_measurement(self) -> None:
        from hball_ti.travel_timer import TravelTimer

        timer = TravelTimer(
            start_mm=-50,
            end_mm=50,
            tolerance_mm=5,
            stable_frames=3,
        )
        samples = (-50, -50, -50, -40, 0, 40, 47)
        measurement = None
        for index, position in enumerate(samples):
            measurement = timer.update(
                position_mm=position,
                confidence=0.9,
                valid=True,
                capture_timestamp_us=index * 100_000,
            )
        self.assertIsNotNone(measurement)
        assert measurement is not None
        self.assertAlmostEqual(measurement.measured_distance_mm, 90.0)
        self.assertAlmostEqual(measurement.elapsed_s, 0.321429, places=5)
        self.assertAlmostEqual(
            measurement.average_speed_mm_s,
            280.0,
            places=2,
        )

    def test_negative_direction_measurement(self) -> None:
        from hball_ti.travel_timer import TravelTimer

        timer = TravelTimer(
            start_mm=50,
            end_mm=-50,
            tolerance_mm=5,
            stable_frames=2,
        )
        measurement = None
        for index, position in enumerate((50, 50, 40, 0, -40, -47)):
            measurement = timer.update(
                position_mm=position,
                confidence=0.95,
                valid=True,
                capture_timestamp_us=index * 50_000,
            )
        self.assertIsNotNone(measurement)
        assert measurement is not None
        self.assertGreater(measurement.elapsed_s, 0)
        self.assertEqual(timer.state, "DONE")

    def test_low_confidence_does_not_arm(self) -> None:
        from hball_ti.travel_timer import TravelTimer

        timer = TravelTimer(
            start_mm=0,
            end_mm=50,
            tolerance_mm=4,
            stable_frames=3,
        )
        for index in range(10):
            timer.update(
                position_mm=0,
                confidence=0.60,
                valid=True,
                capture_timestamp_us=index * 10_000,
            )
        self.assertEqual(timer.state, "WAIT_START")


if __name__ == "__main__":
    unittest.main()
