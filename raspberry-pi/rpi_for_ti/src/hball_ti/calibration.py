"""Relative tube coordinate and millimetre calibration."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True, slots=True)
class PositionCalibration:
    position_min_mm: float
    position_max_mm: float
    zero_u: float
    camera_matrix: np.ndarray | None
    distortion_coeffs: np.ndarray | None

    @classmethod
    def from_dict(cls, data: dict) -> "PositionCalibration":
        minimum = float(data["position_min_mm"])
        maximum = float(data["position_max_mm"])
        zero_u = float(data["zero_u"])
        if not minimum < 0.0 < maximum:
            raise ValueError("position range must straddle zero")
        if not 0.0 < zero_u < 1.0:
            raise ValueError("zero_u must be within (0, 1)")
        matrix = data.get("camera_matrix")
        distortion = data.get("distortion_coeffs")
        return cls(
            minimum,
            maximum,
            zero_u,
            (
                np.asarray(matrix, dtype=np.float64).reshape(3, 3)
                if matrix is not None
                else None
            ),
            (
                np.asarray(distortion, dtype=np.float64).reshape(-1)
                if distortion is not None
                else None
            ),
        )

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        if self.camera_matrix is None or self.distortion_coeffs is None:
            return frame
        return cv2.undistort(
            frame,
            self.camera_matrix,
            self.distortion_coeffs,
        )

    def position_from_u(self, u: float) -> float:
        if u <= self.zero_u:
            fraction = u / self.zero_u
            return self.position_min_mm + fraction * -self.position_min_mm
        fraction = (u - self.zero_u) / (1.0 - self.zero_u)
        return fraction * self.position_max_mm

    def u_from_position(self, position_mm: float) -> float:
        if position_mm <= 0.0:
            return (
                (position_mm - self.position_min_mm)
                / -self.position_min_mm
                * self.zero_u
            )
        return self.zero_u + (
            position_mm / self.position_max_mm * (1.0 - self.zero_u)
        )
