"""Per-frame tube-relative Blob vision pipeline."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .ball_detector import BallDetection, BlobBallDetector
from .calibration import PositionCalibration
from .tube_reference import TubeReference, TubeReferenceDetector
from .vision_result import VisionResult


@dataclass(frozen=True, slots=True)
class PipelineOutput:
    result: VisionResult
    frame: np.ndarray
    reference: TubeReference
    ball: BallDetection | None


class VisionPipeline:
    def __init__(self, config: dict, calibration: dict) -> None:
        self.calibration = PositionCalibration.from_dict(calibration)
        self.reference_detector = TubeReferenceDetector(
            config["tube_reference"],
            self.calibration.zero_u,
        )
        self.ball_detector = BlobBallDetector(
            config["ball_blob"],
            config["tracking"],
        )
        confidence = config["confidence"]
        self.reference_weight = float(
            confidence.get("reference_weight", 0.35)
        )
        self.ball_weight = float(confidence.get("ball_weight", 0.65))
        weight_sum = self.reference_weight + self.ball_weight
        if weight_sum <= 0:
            raise ValueError("confidence weights must sum to a positive value")
        self.reference_weight /= weight_sum
        self.ball_weight /= weight_sum
        runtime = config["runtime"]
        self.hard_min = float(runtime.get("position_hard_min_mm", -120.0))
        self.hard_max = float(runtime.get("position_hard_max_mm", 120.0))

    def process(
        self,
        frame: np.ndarray,
        sequence: int,
        capture_timestamp_us: int,
    ) -> PipelineOutput:
        started = time.perf_counter()
        corrected = self.calibration.undistort(frame)
        reference = self.reference_detector.process(corrected)
        if not reference.valid or reference.strip is None:
            self.ball_detector.mark_reference_lost()
            result = VisionResult(
                sequence=sequence,
                capture_timestamp_us=capture_timestamp_us,
                position_mm=0.0,
                confidence=0.0,
                ball_center_px=None,
                tube_p_negative_px=reference.p_negative,
                tube_p_positive_px=reference.p_positive,
                tube_reference_valid=False,
                valid=False,
                state="NO_TUBE_REFERENCE",
                process_time_ms=(time.perf_counter() - started) * 1000.0,
            )
            return PipelineOutput(result, corrected, reference, None)

        strip_width = reference.strip.shape[1]

        def position_from_x(strip_x: float) -> float:
            u = strip_x / max(strip_width - 1, 1)
            return self.calibration.position_from_u(u)

        ball = self.ball_detector.process(reference.strip, position_from_x)
        ball_center = (
            reference.frame_point_from_strip(
                ball.selected.center_x,
                ball.selected.center_y,
            )
            if ball.selected is not None
            else None
        )
        confidence = (
            self.reference_weight * reference.confidence
            + self.ball_weight * ball.confidence
            if ball.found
            else 0.0
        )
        position = ball.position_mm if ball.found else 0.0
        in_range = self.hard_min <= position <= self.hard_max
        valid = bool(ball.found and in_range)
        if not in_range:
            position = 0.0
            confidence = 0.0
        result = VisionResult(
            sequence=sequence,
            capture_timestamp_us=capture_timestamp_us,
            position_mm=float(position),
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            ball_center_px=ball_center,
            tube_p_negative_px=reference.p_negative,
            tube_p_positive_px=reference.p_positive,
            tube_reference_valid=True,
            valid=valid,
            state=ball.state if in_range else "POSITION_OUT_OF_RANGE",
            process_time_ms=(time.perf_counter() - started) * 1000.0,
        )
        return PipelineOutput(result, corrected, reference, ball)
