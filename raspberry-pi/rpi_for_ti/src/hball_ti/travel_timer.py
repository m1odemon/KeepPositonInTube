"""Robust capture-timestamp timer for travel between two tube positions."""

from __future__ import annotations

from dataclasses import dataclass


UINT32_MASK = 0xFFFFFFFF


def timestamp_delta_us(newer: int, older: int) -> int:
    return (int(newer) - int(older)) & UINT32_MASK


@dataclass(frozen=True, slots=True)
class TravelMeasurement:
    start_mm: float
    end_mm: float
    start_gate_mm: float
    end_gate_mm: float
    start_timestamp_us: int
    end_timestamp_us: int
    elapsed_s: float
    measured_distance_mm: float
    average_speed_mm_s: float
    minimum_confidence: float


class TravelTimer:
    """Arm at the start zone, then time between its two inner boundaries."""

    def __init__(
        self,
        *,
        start_mm: float,
        end_mm: float,
        tolerance_mm: float = 5.0,
        stable_frames: int = 5,
        minimum_confidence: float = 0.70,
        timeout_s: float = 10.0,
        max_lost_ms: float = 200.0,
    ) -> None:
        if start_mm == end_mm:
            raise ValueError("start and end positions must differ")
        if tolerance_mm <= 0:
            raise ValueError("tolerance must be positive")
        if abs(end_mm - start_mm) <= 2.0 * tolerance_mm:
            raise ValueError("point distance must exceed twice the tolerance")
        if stable_frames < 1:
            raise ValueError("stable_frames must be at least 1")
        self.start_mm = float(start_mm)
        self.end_mm = float(end_mm)
        self.tolerance_mm = float(tolerance_mm)
        self.stable_frames = int(stable_frames)
        self.minimum_confidence = float(minimum_confidence)
        self.timeout_us = int(float(timeout_s) * 1_000_000)
        self.max_lost_us = int(float(max_lost_ms) * 1_000)
        self.direction = 1.0 if end_mm > start_mm else -1.0
        self.start_gate_mm = (
            self.start_mm + self.direction * self.tolerance_mm
        )
        self.end_gate_mm = (
            self.end_mm - self.direction * self.tolerance_mm
        )
        self.state = "WAIT_START"
        self.stable_count = 0
        self.previous_position: float | None = None
        self.previous_timestamp_us: int | None = None
        self.start_timestamp_us: int | None = None
        self.last_valid_timestamp_us: int | None = None
        self.minimum_seen_confidence = 1.0
        self.last_event = "place ball in start zone"

    def reset(self, message: str = "reset") -> None:
        self.state = "WAIT_START"
        self.stable_count = 0
        self.previous_position = None
        self.previous_timestamp_us = None
        self.start_timestamp_us = None
        self.last_valid_timestamp_us = None
        self.minimum_seen_confidence = 1.0
        self.last_event = message

    def _accepted(self, valid: bool, confidence: float) -> bool:
        return bool(valid and confidence >= self.minimum_confidence)

    def _inside_start(self, position_mm: float) -> bool:
        return abs(position_mm - self.start_mm) <= self.tolerance_mm

    def _crossed(
        self,
        previous: float,
        current: float,
        gate: float,
    ) -> bool:
        if self.direction > 0:
            return previous < gate <= current
        return previous > gate >= current

    @staticmethod
    def _interpolate_timestamp(
        previous_position: float,
        current_position: float,
        gate: float,
        previous_timestamp_us: int,
        current_timestamp_us: int,
    ) -> int:
        movement = current_position - previous_position
        if abs(movement) < 1e-9:
            return int(current_timestamp_us) & UINT32_MASK
        fraction = (gate - previous_position) / movement
        fraction = min(1.0, max(0.0, fraction))
        delta = timestamp_delta_us(
            current_timestamp_us,
            previous_timestamp_us,
        )
        return (
            int(previous_timestamp_us) + int(round(fraction * delta))
        ) & UINT32_MASK

    def update(
        self,
        *,
        position_mm: float | None,
        confidence: float,
        valid: bool,
        capture_timestamp_us: int,
    ) -> TravelMeasurement | None:
        accepted = (
            position_mm is not None
            and self._accepted(valid, confidence)
        )

        if self.state in {"WAIT_START", "DONE"}:
            if accepted and self._inside_start(float(position_mm)):
                self.stable_count += 1
                self.previous_position = float(position_mm)
                self.previous_timestamp_us = capture_timestamp_us
                if self.stable_count >= self.stable_frames:
                    self.state = "READY"
                    self.last_event = "armed; move ball toward end"
            else:
                self.stable_count = 0
            return None

        if self.state == "READY":
            if not accepted:
                self.reset("start lock lost; place ball at start again")
                return None
            current = float(position_mm)
            if (
                self.previous_position is not None
                and self.previous_timestamp_us is not None
                and self._crossed(
                    self.previous_position,
                    current,
                    self.start_gate_mm,
                )
            ):
                self.start_timestamp_us = self._interpolate_timestamp(
                    self.previous_position,
                    current,
                    self.start_gate_mm,
                    self.previous_timestamp_us,
                    capture_timestamp_us,
                )
                self.last_valid_timestamp_us = capture_timestamp_us
                self.minimum_seen_confidence = confidence
                self.state = "TIMING"
                self.last_event = "timer started"
            elif (
                (current - self.start_mm) * self.direction
                < -self.tolerance_mm
            ):
                self.reset("ball moved away from end; re-arm")
                return None
            self.previous_position = current
            self.previous_timestamp_us = capture_timestamp_us
            return None

        if self.state != "TIMING":
            return None
        assert self.start_timestamp_us is not None

        if not accepted:
            if (
                self.last_valid_timestamp_us is not None
                and timestamp_delta_us(
                    capture_timestamp_us,
                    self.last_valid_timestamp_us,
                )
                > self.max_lost_us
            ):
                self.reset("ball lost during timing; run discarded")
            return None

        current = float(position_mm)
        self.last_valid_timestamp_us = capture_timestamp_us
        self.minimum_seen_confidence = min(
            self.minimum_seen_confidence,
            confidence,
        )
        if (
            timestamp_delta_us(
                capture_timestamp_us,
                self.start_timestamp_us,
            )
            > self.timeout_us
        ):
            self.reset("travel timeout; run discarded")
            return None

        if (
            self.previous_position is not None
            and self.previous_timestamp_us is not None
            and self._crossed(
                self.previous_position,
                current,
                self.end_gate_mm,
            )
        ):
            end_timestamp = self._interpolate_timestamp(
                self.previous_position,
                current,
                self.end_gate_mm,
                self.previous_timestamp_us,
                capture_timestamp_us,
            )
            elapsed_s = (
                timestamp_delta_us(
                    end_timestamp,
                    self.start_timestamp_us,
                )
                / 1_000_000.0
            )
            distance = abs(self.end_gate_mm - self.start_gate_mm)
            measurement = TravelMeasurement(
                self.start_mm,
                self.end_mm,
                self.start_gate_mm,
                self.end_gate_mm,
                self.start_timestamp_us,
                end_timestamp,
                elapsed_s,
                distance,
                distance / elapsed_s if elapsed_s > 0 else float("inf"),
                self.minimum_seen_confidence,
            )
            self.state = "DONE"
            self.stable_count = 0
            self.last_event = f"done: {elapsed_s:.4f} s"
            return measurement

        self.previous_position = current
        self.previous_timestamp_us = capture_timestamp_us
        return None
