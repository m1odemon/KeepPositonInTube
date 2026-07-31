"""Measure steel-ball travel time with the dynamic-ROI Blob detector."""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2

from hball_ti.camera_source import LatestFrameCamera
from hball_ti.config import load_json, load_yaml
from hball_ti.debug_view import draw_debug
from hball_ti.travel_timer import TravelMeasurement, TravelTimer
from hball_ti.vision_pipeline import PipelineOutput, VisionPipeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Time a real ball travelling between two tube positions",
    )
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--calibration", default="calibration.json")
    parser.add_argument("--start-mm", type=float, required=True)
    parser.add_argument("--end-mm", type=float, required=True)
    parser.add_argument("--tolerance-mm", type=float, default=5.0)
    parser.add_argument("--stable-frames", type=int, default=5)
    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--timeout-s", type=float, default=10.0)
    parser.add_argument("--max-lost-ms", type=float, default=200.0)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--no-csv", action="store_true")
    parser.add_argument("--csv", default=None)
    return parser.parse_args()


def raw_ball_position(
    output: PipelineOutput,
    pipeline: VisionPipeline,
) -> float | None:
    if (
        not output.result.valid
        or output.ball is None
        or not output.ball.found
        or output.ball.selected is None
        or output.reference.strip is None
    ):
        return None
    width = output.reference.strip.shape[1]
    u = output.ball.selected.center_x / max(width - 1, 1)
    return pipeline.calibration.position_from_u(u)


def write_measurement(
    writer: csv.writer | None,
    run_number: int,
    measurement: TravelMeasurement,
) -> None:
    row = (
        datetime.now(timezone.utc).isoformat(),
        run_number,
        f"{measurement.start_mm:.3f}",
        f"{measurement.end_mm:.3f}",
        f"{measurement.start_gate_mm:.3f}",
        f"{measurement.end_gate_mm:.3f}",
        measurement.start_timestamp_us,
        measurement.end_timestamp_us,
        f"{measurement.elapsed_s:.6f}",
        f"{measurement.measured_distance_mm:.3f}",
        f"{measurement.average_speed_mm_s:.3f}",
        f"{measurement.minimum_confidence:.4f}",
    )
    if writer is not None:
        writer.writerow(row)
    print(
        f"RUN {run_number}: time={measurement.elapsed_s:.4f}s "
        f"distance={measurement.measured_distance_mm:.1f}mm "
        f"average_speed={measurement.average_speed_mm_s:.1f}mm/s "
        f"min_confidence={measurement.minimum_confidence:.2f}"
    )


def main() -> int:
    args = parse_args()
    if args.runs < 1:
        raise ValueError("--runs must be at least 1")
    config_path = Path(args.config).resolve()
    calibration_path = Path(args.calibration).resolve()
    config = load_yaml(config_path)
    calibration_data = load_json(calibration_path)
    pipeline = VisionPipeline(config, calibration_data)
    camera = LatestFrameCamera.from_config(config)
    timer = TravelTimer(
        start_mm=args.start_mm,
        end_mm=args.end_mm,
        tolerance_mm=args.tolerance_mm,
        stable_frames=args.stable_frames,
        minimum_confidence=args.min_confidence,
        timeout_s=args.timeout_s,
        max_lost_ms=args.max_lost_ms,
    )

    csv_file = None
    writer = None
    csv_path = None
    if not args.no_csv:
        if args.csv:
            csv_path = Path(args.csv)
            if not csv_path.is_absolute():
                csv_path = config_path.parent / csv_path
        else:
            log_root = config_path.parent / config["runtime"].get(
                "log_directory",
                "logs",
            )
            csv_path = log_root / (
                "travel_time_"
                + datetime.now().strftime("%Y%m%d_%H%M%S")
                + ".csv"
            )
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow(
            (
                "host_time",
                "run",
                "start_mm",
                "end_mm",
                "start_gate_mm",
                "end_gate_mm",
                "start_capture_timestamp_us",
                "end_capture_timestamp_us",
                "elapsed_s",
                "measured_distance_mm",
                "average_speed_mm_s",
                "minimum_confidence",
            )
        )

    sequence = 0
    camera_sequence = 0
    completed_runs = 0
    report_started = time.monotonic()
    report_frames = 0
    fps = 0.0
    finish_deadline = None
    print(
        f"Travel test: {args.start_mm:+.1f} -> {args.end_mm:+.1f} mm; "
        f"timer gates {timer.start_gate_mm:+.1f} -> "
        f"{timer.end_gate_mm:+.1f} mm"
    )
    print("Place the ball at START; timing begins after it leaves the zone.")

    try:
        camera.open()
        while True:
            captured = camera.read_after(camera_sequence, timeout=0.5)
            if captured is None:
                raise RuntimeError("camera delivered no new frame for 0.5 s")
            camera_sequence = captured.sequence
            output = pipeline.process(
                captured.image,
                sequence,
                captured.capture_timestamp_us,
            )
            sequence = (sequence + 1) & 0xFFFFFFFF
            raw_position = raw_ball_position(output, pipeline)
            measurement = timer.update(
                position_mm=raw_position,
                confidence=output.result.confidence,
                valid=output.result.valid,
                capture_timestamp_us=captured.capture_timestamp_us,
            )
            if measurement is not None:
                completed_runs += 1
                write_measurement(writer, completed_runs, measurement)
                if csv_file is not None:
                    csv_file.flush()
                if completed_runs >= args.runs:
                    if args.no_show:
                        break
                    finish_deadline = time.monotonic() + 2.0

            report_frames += 1
            elapsed = time.monotonic() - report_started
            if elapsed >= 1.0:
                fps = report_frames / elapsed
                report_frames = 0
                report_started = time.monotonic()
                print(
                    f"fps={fps:.1f} state={timer.state} "
                    f"raw={raw_position} "
                    f"confidence={output.result.confidence:.2f} "
                    f"event={timer.last_event}"
                )

            if not args.no_show:
                debug = draw_debug(
                    output,
                    fps=fps,
                    frame_period_ms=0.0,
                    uart_write_ms=0.0,
                    ti_threshold=args.min_confidence,
                )
                # Use the pipeline calibration directly for the two gate lines.
                strip = output.reference.strip
                if strip is not None:
                    for value, color, label in (
                        (timer.start_gate_mm, (0, 200, 255), "START"),
                        (timer.end_gate_mm, (255, 0, 255), "END"),
                    ):
                        u = pipeline.calibration.u_from_position(value)
                        x = int(round(u * (debug.shape[1] - 1)))
                        cv2.line(
                            debug,
                            (x, output.frame.shape[0]),
                            (x, debug.shape[0] - 1),
                            color,
                            2,
                        )
                        cv2.putText(
                            debug,
                            label,
                            (max(2, x - 22), output.frame.shape[0] + 18),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            color,
                            1,
                        )
                position_text = (
                    "none"
                    if raw_position is None
                    else f"{raw_position:+.1f}mm"
                )
                cv2.rectangle(
                    debug,
                    (0, 0),
                    (debug.shape[1], 66),
                    (0, 0, 0),
                    -1,
                )
                cv2.putText(
                    debug,
                    (
                        f"TIMER={timer.state} raw={position_text} "
                        f"runs={completed_runs}/{args.runs}"
                    ),
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug,
                    timer.last_event,
                    (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                cv2.imshow("Blob ball travel-time test", debug)
                key = cv2.waitKey(1) & 0xFF
                if key in {ord("q"), ord("Q"), 27}:
                    break
                if key in {ord("r"), ord("R")}:
                    timer.reset("manual reset")
                    finish_deadline = None
            if finish_deadline is not None and time.monotonic() >= finish_deadline:
                break
    finally:
        camera.close()
        if csv_file is not None:
            csv_file.close()
            print(f"CSV: {csv_path}")
        if not args.no_show:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
