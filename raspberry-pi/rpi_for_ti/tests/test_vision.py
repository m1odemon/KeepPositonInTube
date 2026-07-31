from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))


def synthetic_frame(angle_deg: float = 0.0) -> np.ndarray:
    frame = np.full((480, 640, 3), 185, dtype=np.uint8)
    center = np.asarray([320.0, 240.0])
    half_axis = 250.0
    radians = math.radians(angle_deg)
    unit = np.asarray([math.cos(radians), math.sin(radians)])
    p_negative = center - half_axis * unit
    p_positive = center + half_axis * unit
    cv2.line(
        frame,
        tuple(np.rint(p_negative).astype(int)),
        tuple(np.rint(p_positive).astype(int)),
        (225, 225, 225),
        96,
        cv2.LINE_AA,
    )
    cv2.ellipse(
        frame,
        tuple(np.rint(center).astype(int)),
        (18, 38),
        angle_deg,
        0,
        360,
        (18, 18, 18),
        -1,
        cv2.LINE_AA,
    )
    cv2.ellipse(
        frame,
        tuple(np.rint(center + np.asarray([4.0, -5.0])).astype(int)),
        (6, 12),
        angle_deg,
        0,
        360,
        (245, 245, 245),
        -1,
        cv2.LINE_AA,
    )
    cv2.circle(
        frame,
        tuple(np.rint(p_negative).astype(int)),
        15,
        (0, 0, 255),
        -1,
        cv2.LINE_AA,
    )
    cv2.circle(
        frame,
        tuple(np.rint(p_positive).astype(int)),
        15,
        (255, 0, 0),
        -1,
        cv2.LINE_AA,
    )
    return frame


@unittest.skipIf(cv2 is None, "OpenCV is required")
class VisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cv2.setNumThreads(1)
        cls.config = yaml.safe_load(
            (ROOT / "config.yaml").read_text(encoding="utf-8")
        )
        import json

        cls.calibration = json.loads(
            (ROOT / "calibration.json").read_text(encoding="utf-8")
        )

    def test_dynamic_tube_reference(self) -> None:
        from hball_ti.tube_reference import TubeReferenceDetector

        detector = TubeReferenceDetector(
            self.config["tube_reference"],
            zero_u=0.5,
        )
        result = detector.process(synthetic_frame(5.0))
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.axis_length_px, 500, delta=8)
        self.assertAlmostEqual(result.angle_deg, 5.0, delta=1.0)
        self.assertIsNotNone(result.strip)
        self.assertEqual(result.strip.shape[:2], (120, 640))
        self.assertGreater(result.confidence, 0.70)

    def test_blob_pipeline_tracks_center_ball(self) -> None:
        from hball_ti.vision_pipeline import VisionPipeline

        pipeline = VisionPipeline(self.config, self.calibration)
        outputs = [
            pipeline.process(synthetic_frame(4.0), index, index * 10_000)
            for index in range(3)
        ]
        self.assertFalse(outputs[0].result.valid)
        self.assertEqual(outputs[0].result.state, "CONFIRM")
        self.assertTrue(outputs[-1].result.valid)
        self.assertAlmostEqual(
            outputs[-1].result.position_mm,
            0.0,
            delta=3.0,
        )
        self.assertGreater(outputs[-1].result.confidence, 0.70)

    def test_position_is_relative_to_tilted_tube(self) -> None:
        from hball_ti.vision_pipeline import VisionPipeline

        positions = []
        for angle in (-5.0, 0.0, 5.0):
            pipeline = VisionPipeline(self.config, self.calibration)
            output = None
            for index in range(3):
                output = pipeline.process(
                    synthetic_frame(angle),
                    index,
                    index * 10_000,
                )
            assert output is not None
            self.assertTrue(output.result.valid)
            positions.append(output.result.position_mm)
        self.assertLess(max(positions) - min(positions), 3.0)

    def test_missing_marker_forces_zero_confidence(self) -> None:
        from hball_ti.vision_pipeline import VisionPipeline

        frame = synthetic_frame()
        frame[:, 500:] = 185
        output = VisionPipeline(
            self.config,
            self.calibration,
        ).process(frame, 0, 1000)
        self.assertFalse(output.result.valid)
        self.assertFalse(output.result.tube_reference_valid)
        self.assertEqual(output.result.position_mm, 0.0)
        self.assertEqual(output.result.confidence, 0.0)

    def test_three_missing_ball_frames_enter_lost_state(self) -> None:
        from hball_ti.vision_pipeline import VisionPipeline

        pipeline = VisionPipeline(self.config, self.calibration)
        for index in range(3):
            pipeline.process(
                synthetic_frame(),
                index,
                index * 10_000,
            )
        empty_tube = synthetic_frame()
        cv2.rectangle(empty_tube, (290, 190), (350, 290), (225, 225, 225), -1)
        output = None
        for index in range(3, 6):
            output = pipeline.process(
                empty_tube,
                index,
                index * 10_000,
            )
        assert output is not None
        self.assertFalse(output.result.valid)
        self.assertEqual(output.result.state, "LOST")
        self.assertEqual(output.result.confidence, 0.0)


if __name__ == "__main__":
    unittest.main()
