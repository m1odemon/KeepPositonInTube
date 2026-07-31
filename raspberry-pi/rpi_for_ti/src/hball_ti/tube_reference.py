"""Detect coloured tube-end references and build a per-frame tube ROI."""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True, slots=True)
class MarkerDetection:
    found: bool
    center: tuple[float, float] | None
    area: int
    quality: float
    mask: np.ndarray


@dataclass(frozen=True, slots=True)
class TubeReference:
    valid: bool
    p_negative: tuple[float, float] | None
    p_positive: tuple[float, float] | None
    zero_point: tuple[float, float] | None
    angle_deg: float
    axis_length_px: float
    confidence: float
    strip: np.ndarray | None
    to_strip: np.ndarray | None
    to_frame: np.ndarray | None
    negative_mask: np.ndarray
    positive_mask: np.ndarray

    def frame_point_from_strip(
        self,
        strip_x: float,
        strip_y: float,
    ) -> tuple[float, float] | None:
        if self.to_frame is None:
            return None
        point = np.asarray([[[strip_x, strip_y]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(point, self.to_frame)[0, 0]
        return float(mapped[0]), float(mapped[1])


class TubeReferenceDetector:
    def __init__(self, settings: dict, zero_u: float) -> None:
        self.negative_ranges = tuple(settings["p_negative_hsv_ranges"])
        self.positive_ranges = tuple(settings["p_positive_hsv_ranges"])
        self.kernel_size = int(settings.get("morphology_kernel", 5))
        self.area_min = int(settings.get("marker_area_min", 80))
        self.area_max = int(settings.get("marker_area_max", 5000))
        self.expected_area = float(settings.get("expected_marker_area", 700))
        self.axis_min = float(settings.get("axis_length_min_px", 260))
        self.axis_max = float(settings.get("axis_length_max_px", 620))
        self.expected_axis = float(
            settings.get("expected_axis_length_px", 500)
        )
        self.max_angle = float(settings.get("max_abs_angle_deg", 20))
        self.jump_gate = float(settings.get("endpoint_jump_gate_px", 80))
        self.strip_width, self.strip_height = (
            int(value) for value in settings.get("strip_size", [640, 120])
        )
        self.half_width = float(settings.get("tube_half_width_px", 48))
        self.zero_u = float(zero_u)
        self.previous_negative: np.ndarray | None = None
        self.previous_positive: np.ndarray | None = None

    @staticmethod
    def _ratio_quality(value: float, expected: float) -> float:
        if value <= 0 or expected <= 0:
            return 0.0
        return math.exp(-abs(math.log(value / expected)))

    def _mask(self, hsv: np.ndarray, ranges: tuple) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for values in ranges:
            if len(values) != 6:
                raise ValueError("HSV range must contain six integers")
            low = np.asarray(values[:3], dtype=np.uint8)
            high = np.asarray(values[3:], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.kernel_size, self.kernel_size),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    def _marker(self, mask: np.ndarray) -> MarkerDetection:
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        valid: list[tuple[int, int]] = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if self.area_min <= area <= self.area_max:
                valid.append((area, label))
        if not valid:
            return MarkerDetection(False, None, 0, 0.0, mask)
        area, label = max(valid)
        center = (
            float(centroids[label][0]),
            float(centroids[label][1]),
        )
        return MarkerDetection(
            True,
            center,
            area,
            self._ratio_quality(area, self.expected_area),
            mask,
        )

    def process(self, frame: np.ndarray) -> TubeReference:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        negative = self._marker(self._mask(hsv, self.negative_ranges))
        positive = self._marker(self._mask(hsv, self.positive_ranges))
        empty = np.zeros(frame.shape[:2], dtype=np.uint8)
        if not negative.found or not positive.found:
            return TubeReference(
                False,
                negative.center,
                positive.center,
                None,
                0.0,
                0.0,
                0.0,
                None,
                None,
                None,
                negative.mask if negative.mask is not None else empty,
                positive.mask if positive.mask is not None else empty,
            )

        p_negative = np.asarray(negative.center, dtype=np.float32)
        p_positive = np.asarray(positive.center, dtype=np.float32)
        axis = p_positive - p_negative
        length = float(np.linalg.norm(axis))
        angle = math.degrees(math.atan2(float(axis[1]), float(axis[0])))
        jump_quality = 1.0
        if (
            self.previous_negative is not None
            and self.previous_positive is not None
        ):
            jump = max(
                float(np.linalg.norm(p_negative - self.previous_negative)),
                float(np.linalg.norm(p_positive - self.previous_positive)),
            )
            jump_quality = max(0.0, 1.0 - jump / max(self.jump_gate, 1.0))
            if jump > self.jump_gate:
                return TubeReference(
                    False,
                    negative.center,
                    positive.center,
                    None,
                    angle,
                    length,
                    0.0,
                    None,
                    None,
                    None,
                    negative.mask,
                    positive.mask,
                )
        if not self.axis_min <= length <= self.axis_max:
            return TubeReference(
                False,
                negative.center,
                positive.center,
                None,
                angle,
                length,
                0.0,
                None,
                None,
                None,
                negative.mask,
                positive.mask,
            )
        horizontal_angle = min(abs(angle), abs(abs(angle) - 180.0))
        if horizontal_angle > self.max_angle:
            return TubeReference(
                False,
                negative.center,
                positive.center,
                None,
                angle,
                length,
                0.0,
                None,
                None,
                None,
                negative.mask,
                positive.mask,
            )

        unit = axis / length
        normal = np.asarray([-unit[1], unit[0]], dtype=np.float32)
        source = np.asarray(
            [
                p_negative - normal * self.half_width,
                p_positive - normal * self.half_width,
                p_positive + normal * self.half_width,
                p_negative + normal * self.half_width,
            ],
            dtype=np.float32,
        )
        destination = np.asarray(
            [
                [0, 0],
                [self.strip_width - 1, 0],
                [self.strip_width - 1, self.strip_height - 1],
                [0, self.strip_height - 1],
            ],
            dtype=np.float32,
        )
        to_strip = cv2.getPerspectiveTransform(source, destination)
        to_frame = cv2.getPerspectiveTransform(destination, source)
        strip = cv2.warpPerspective(
            frame,
            to_strip,
            (self.strip_width, self.strip_height),
        )
        angle_quality = max(0.0, 1.0 - horizontal_angle / self.max_angle)
        length_quality = self._ratio_quality(length, self.expected_axis)
        confidence = float(
            np.clip(
                0.35 * (negative.quality + positive.quality) / 2.0
                + 0.30 * length_quality
                + 0.20 * angle_quality
                + 0.15 * jump_quality,
                0.0,
                1.0,
            )
        )
        self.previous_negative = p_negative
        self.previous_positive = p_positive
        zero = p_negative + self.zero_u * axis
        return TubeReference(
            True,
            negative.center,
            positive.center,
            (float(zero[0]), float(zero[1])),
            angle,
            length,
            confidence,
            strip,
            to_strip,
            to_frame,
            negative.mask,
            positive.mask,
        )
