"""CSV logger for TI integration evidence."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path

from .vision_result import VisionResult


class SessionLogger:
    HEADER = (
        "host_time",
        "sequence",
        "capture_timestamp_us",
        "ball_pixel_x",
        "ball_pixel_y",
        "tube_p_negative_x",
        "tube_p_negative_y",
        "tube_p_positive_x",
        "tube_p_positive_y",
        "tube_reference_valid",
        "position_mm",
        "confidence",
        "vision_valid",
        "state",
        "frame_period_ms",
        "process_time_ms",
        "uart_write_time_ms",
    )

    def __init__(self, directory: str | Path) -> None:
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = root / f"ti_vision_{timestamp}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._rows_since_flush = 0

    @staticmethod
    def _xy(
        value: tuple[float, float] | None,
    ) -> tuple[str, str]:
        if value is None:
            return "", ""
        return f"{value[0]:.3f}", f"{value[1]:.3f}"

    def write(
        self,
        result: VisionResult,
        frame_period_ms: float,
        uart_write_ms: float,
    ) -> None:
        ball_x, ball_y = self._xy(result.ball_center_px)
        negative_x, negative_y = self._xy(result.tube_p_negative_px)
        positive_x, positive_y = self._xy(result.tube_p_positive_px)
        self._writer.writerow(
            (
                datetime.now(timezone.utc).isoformat(),
                result.sequence,
                result.capture_timestamp_us,
                ball_x,
                ball_y,
                negative_x,
                negative_y,
                positive_x,
                positive_y,
                int(result.tube_reference_valid),
                f"{result.position_mm:.4f}",
                f"{result.confidence:.5f}",
                int(result.valid),
                result.state,
                f"{frame_period_ms:.3f}",
                f"{result.process_time_ms:.3f}",
                f"{uart_write_ms:.3f}",
            )
        )
        self._rows_since_flush += 1
        if self._rows_since_flush >= 30:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        self._file.flush()
        self._file.close()
