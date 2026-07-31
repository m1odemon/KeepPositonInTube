"""Blob detector tuned from the real replacement-tube steel ball."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import cv2
import numpy as np


@dataclass(frozen=True, slots=True)
class BallCandidate:
    x: int
    y: int
    width: int
    height: int
    center_x: float
    center_y: float
    area: int
    aspect_ratio: float
    circularity: float
    solidity: float
    bbox_fill_ratio: float
    internal_fill_ratio: float
    score: float


@dataclass(frozen=True, slots=True)
class BallDetection:
    found: bool
    position_mm: float
    confidence: float
    state: str
    selected: BallCandidate | None
    candidates: tuple[BallCandidate, ...]
    gray: np.ndarray
    binary: np.ndarray


class BlobBallDetector:
    def __init__(self, settings: dict, tracking: dict) -> None:
        for key, value in settings.items():
            setattr(self, key, value)
        self.acquire_frames = max(
            1,
            int(tracking.get("acquire_valid_frames", 3)),
        )
        self.lost_after_frames = max(
            1,
            int(tracking.get("lost_after_frames", 3)),
        )
        self.jump_gate_mm = float(
            tracking.get("position_jump_gate_mm", 30)
        )
        self.alpha = float(
            np.clip(tracking.get("smoothing_alpha", 0.30), 0.0, 1.0)
        )
        self.state = "SEARCH"
        self.pending_position: float | None = None
        self.pending_hits = 0
        self.filtered_position: float | None = None
        self.missed_frames = 0

    @staticmethod
    def _ratio_quality(value: float, expected: float) -> float:
        if value <= 0 or expected <= 0:
            return 0.0
        return math.exp(-abs(math.log(value / expected)))

    def _candidate(
        self,
        label: int,
        labels: np.ndarray,
        stats: np.ndarray,
        centroids: np.ndarray,
    ) -> BallCandidate | None:
        x, y, width, height, area = (int(value) for value in stats[label])
        if not int(self.min_area) <= area <= int(self.max_area):
            return None
        if not int(self.min_width) <= width <= int(self.max_width):
            return None
        if not int(self.min_height) <= height <= int(self.max_height):
            return None
        if (
            x < int(self.edge_margin_px)
            or x + width > labels.shape[1] - int(self.edge_margin_px)
        ):
            return None
        center_x = float(centroids[label][0])
        center_y = float(centroids[label][1])
        vertical_offset = abs(center_y - labels.shape[0] / 2.0)
        if vertical_offset > float(self.max_vertical_offset_px):
            return None
        aspect = width / max(height, 1)
        if not float(self.min_aspect_ratio) <= aspect <= float(
            self.max_aspect_ratio
        ):
            return None

        component = np.zeros((height, width), dtype=np.uint8)
        component[labels[y : y + height, x : x + width] == label] = 255
        contours, _ = cv2.findContours(
            component,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        contour_area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        if contour_area <= 0 or perimeter <= 0:
            return None
        circularity = 4.0 * math.pi * contour_area / (perimeter * perimeter)
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = contour_area / hull_area if hull_area > 0 else 0.0
        bbox_fill = area / float(width * height)
        outer_mask = np.zeros_like(component)
        cv2.drawContours(outer_mask, [contour], -1, 255, cv2.FILLED)
        outer_pixels = int(cv2.countNonZero(outer_mask))
        internal_fill = area / outer_pixels if outer_pixels else 0.0
        if circularity < float(self.min_circularity):
            return None
        if solidity < float(self.min_solidity):
            return None
        if not float(self.min_bbox_fill_ratio) <= bbox_fill <= float(
            self.max_bbox_fill_ratio
        ):
            return None
        if internal_fill < float(self.min_internal_fill_ratio):
            return None

        vertical_quality = max(
            0.0,
            1.0
            - vertical_offset / max(float(self.max_vertical_offset_px), 1.0),
        )
        score = float(
            np.clip(
                0.24 * self._ratio_quality(area, float(self.expected_area))
                + 0.14
                * self._ratio_quality(width, float(self.expected_width))
                + 0.14
                * self._ratio_quality(height, float(self.expected_height))
                + 0.14
                * self._ratio_quality(
                    aspect,
                    float(self.expected_aspect_ratio),
                )
                + 0.10 * min(1.0, solidity)
                + 0.08 * min(1.0, bbox_fill)
                + 0.06 * min(1.0, internal_fill)
                + 0.04 * min(1.0, circularity)
                + 0.06 * vertical_quality,
                0.0,
                1.0,
            )
        )
        if score < float(self.min_candidate_confidence):
            return None
        return BallCandidate(
            x,
            y,
            width,
            height,
            center_x,
            center_y,
            area,
            aspect,
            circularity,
            solidity,
            bbox_fill,
            internal_fill,
            score,
        )

    def _register_miss(self) -> None:
        self.missed_frames += 1
        if self.missed_frames >= self.lost_after_frames:
            self.state = "LOST"
            self.filtered_position = None
            self.pending_position = None
            self.pending_hits = 0
        elif self.state != "TRACK":
            self.pending_position = None
            self.pending_hits = 0

    def mark_reference_lost(self) -> None:
        """Advance loss state when no valid per-frame tube axis exists."""
        self._register_miss()

    def process(
        self,
        strip: np.ndarray,
        position_from_strip_x: Callable[[float], float],
    ) -> BallDetection:
        gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY)
        blur = int(self.blur_kernel)
        gray = cv2.GaussianBlur(gray, (blur, blur), 0)
        threshold_type = (
            cv2.THRESH_BINARY_INV
            if str(self.polarity).lower() == "dark"
            else cv2.THRESH_BINARY
        )
        threshold_value = int(self.threshold)
        if bool(self.use_otsu):
            threshold_type |= cv2.THRESH_OTSU
            threshold_value = 0
        _, binary = cv2.threshold(
            gray,
            threshold_value,
            255,
            threshold_type,
        )
        morphology = int(self.morphology_kernel)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (morphology, morphology),
        )
        if int(self.open_iterations) > 0:
            binary = cv2.morphologyEx(
                binary,
                cv2.MORPH_OPEN,
                kernel,
                iterations=int(self.open_iterations),
            )
        if int(self.close_iterations) > 0:
            binary = cv2.morphologyEx(
                binary,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=int(self.close_iterations),
            )
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary,
            8,
        )
        candidates = tuple(
            candidate
            for label in range(1, count)
            if (
                candidate := self._candidate(
                    label,
                    labels,
                    stats,
                    centroids,
                )
            )
            is not None
        )
        ranked: list[tuple[float, BallCandidate, float]] = []
        for candidate in candidates:
            position = position_from_strip_x(candidate.center_x)
            if (
                self.state == "TRACK"
                and self.filtered_position is not None
                and abs(position - self.filtered_position) > self.jump_gate_mm
            ):
                continue
            ranked.append((candidate.score, candidate, position))
        if not ranked:
            self._register_miss()
            return BallDetection(
                False,
                0.0,
                0.0,
                self.state,
                None,
                candidates,
                gray,
                binary,
            )
        ranked.sort(key=lambda item: item[0], reverse=True)
        confidence, selected, position = ranked[0]
        if len(ranked) > 1:
            second_score, second, _ = ranked[1]
            if (
                abs(second.center_x - selected.center_x)
                > float(self.ambiguity_distance_px)
                and second_score > 0.85 * confidence
            ):
                confidence *= 0.65

        if self.state != "TRACK":
            if (
                self.pending_position is None
                or abs(position - self.pending_position) > self.jump_gate_mm
            ):
                self.pending_position = position
                self.pending_hits = 1
            else:
                self.pending_position = (
                    0.6 * self.pending_position + 0.4 * position
                )
                self.pending_hits += 1
            self.missed_frames = 0
            if self.pending_hits < self.acquire_frames:
                self.state = "CONFIRM"
                return BallDetection(
                    False,
                    0.0,
                    0.0,
                    self.state,
                    selected,
                    candidates,
                    gray,
                    binary,
                )
            self.filtered_position = float(self.pending_position)
            self.pending_position = None
            self.pending_hits = 0
        else:
            assert self.filtered_position is not None
            self.filtered_position = (
                self.alpha * position
                + (1.0 - self.alpha) * self.filtered_position
            )
        self.state = "TRACK"
        self.missed_frames = 0
        return BallDetection(
            True,
            float(self.filtered_position),
            float(np.clip(confidence, 0.0, 1.0)),
            self.state,
            selected,
            candidates,
            gray,
            binary,
        )
