"""Single-source-of-truth result passed to logging and UART."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class VisionResult:
    sequence: int
    capture_timestamp_us: int
    position_mm: float
    confidence: float
    ball_center_px: tuple[float, float] | None
    tube_p_negative_px: tuple[float, float] | None
    tube_p_positive_px: tuple[float, float] | None
    tube_reference_valid: bool
    valid: bool
    state: str
    process_time_ms: float
