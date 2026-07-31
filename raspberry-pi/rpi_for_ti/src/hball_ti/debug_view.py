"""Diagnostic overlay used only during offline and integration testing."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .vision_pipeline import PipelineOutput


def _point(value: tuple[float, float]) -> tuple[int, int]:
    return int(round(value[0])), int(round(value[1]))


def draw_debug(
    output: PipelineOutput,
    *,
    fps: float,
    frame_period_ms: float,
    uart_write_ms: float,
    ti_threshold: float,
) -> np.ndarray:
    frame = output.frame.copy()
    result = output.result
    reference = output.reference
    if reference.strip is not None:
        roi_points = [
            reference.frame_point_from_strip(0, 0),
            reference.frame_point_from_strip(reference.strip.shape[1] - 1, 0),
            reference.frame_point_from_strip(
                reference.strip.shape[1] - 1,
                reference.strip.shape[0] - 1,
            ),
            reference.frame_point_from_strip(
                0,
                reference.strip.shape[0] - 1,
            ),
        ]
        if all(point is not None for point in roi_points):
            polygon = np.asarray(
                [_point(point) for point in roi_points if point is not None],
                dtype=np.int32,
            )
            cv2.polylines(
                frame,
                [polygon],
                True,
                (255, 180, 0),
                1,
                cv2.LINE_AA,
            )
    if reference.p_negative is not None:
        point = _point(reference.p_negative)
        cv2.circle(frame, point, 8, (0, 0, 255), 2)
        cv2.putText(
            frame,
            "P-",
            (point[0] + 8, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
    if reference.p_positive is not None:
        point = _point(reference.p_positive)
        cv2.circle(frame, point, 8, (255, 0, 0), 2)
        cv2.putText(
            frame,
            "P+",
            (point[0] + 8, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 0, 0),
            2,
        )
    if (
        reference.p_negative is not None
        and reference.p_positive is not None
    ):
        cv2.line(
            frame,
            _point(reference.p_negative),
            _point(reference.p_positive),
            (0, 220, 220),
            2,
            cv2.LINE_AA,
        )
    if reference.zero_point is not None:
        point = _point(reference.zero_point)
        cv2.circle(frame, point, 6, (0, 255, 255), -1)
        cv2.putText(
            frame,
            "O",
            (point[0] + 8, point[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
    if result.ball_center_px is not None:
        color = (0, 255, 0) if result.valid else (0, 180, 255)
        cv2.circle(frame, _point(result.ball_center_px), 12, color, 2)

    valid_for_ti = result.valid and result.confidence >= ti_threshold
    lines = [
        (
            f"state={result.state} valid={result.valid} "
            f"TI_ACCEPT={valid_for_ti}"
        ),
        (
            f"position={result.position_mm:+.1f} mm "
            f"confidence={result.confidence:.2f} "
            f"tube={reference.confidence:.2f}"
        ),
        (
            f"FPS={fps:.1f} period={frame_period_ms:.1f} ms "
            f"process={result.process_time_ms:.1f} ms "
            f"uart={uart_write_ms:.2f} ms"
        ),
    ]
    for index, text in enumerate(lines):
        cv2.putText(
            frame,
            text,
            (10, 26 + index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (20, 255, 255),
            2,
            cv2.LINE_AA,
        )

    strip_width = 640
    strip_height = 120
    if reference.strip is None:
        strip = np.zeros((strip_height, strip_width, 3), dtype=np.uint8)
        cv2.putText(
            strip,
            "NO VALID TUBE REFERENCE",
            (12, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
    else:
        strip = reference.strip.copy()
        if output.ball is not None and output.ball.selected is not None:
            candidate = output.ball.selected
            equivalent_radius = math.sqrt(candidate.area / math.pi)
            color = (0, 255, 0) if output.ball.found else (0, 180, 255)
            cv2.rectangle(
                strip,
                (candidate.x, candidate.y),
                (
                    candidate.x + candidate.width,
                    candidate.y + candidate.height,
                ),
                color,
                2,
            )
            cv2.putText(
                strip,
                (
                    f"A={candidate.area} W={candidate.width} "
                    f"H={candidate.height} R_eq={equivalent_radius:.1f} "
                    f"score={candidate.score:.2f}"
                ),
                (max(5, candidate.x - 20), max(16, candidate.y - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv2.LINE_AA,
            )
    if strip.shape[1] != frame.shape[1]:
        strip = cv2.resize(strip, (frame.shape[1], strip.shape[0]))
    return np.vstack((frame, strip))


class DebugVideoWriter:
    def __init__(self, path: str, fps: float, frame_size: tuple[int, int]):
        self.writer = cv2.VideoWriter(
            path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            frame_size,
        )
        if not self.writer.isOpened():
            raise RuntimeError(f"cannot open debug video: {path}")

    def write(self, frame: np.ndarray) -> None:
        self.writer.write(frame)

    def close(self) -> None:
        self.writer.release()
