"""Camera -> dynamic tube reference -> Blob -> TI UART main loop."""

from __future__ import annotations

import argparse
import signal
import time
from datetime import datetime
from pathlib import Path

import cv2

from .camera_source import LatestFrameCamera
from .config import load_json, load_yaml
from .debug_view import DebugVideoWriter, draw_debug
from .session_logger import SessionLogger
from .ti_protocol import make_vision_frame
from .ti_serial_link import TISerialLink
from .vision_pipeline import VisionPipeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--calibration", default="calibration.json")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="build and validate every frame but do not open UART",
    )
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--no-csv", action="store_true")
    parser.add_argument(
        "--record-video",
        default=None,
        help="optional debug MP4 path; disabled in competition mode",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).resolve()
    calibration_path = Path(args.calibration).resolve()
    config = load_yaml(config_path)
    calibration = load_json(calibration_path)
    camera = LatestFrameCamera.from_config(config)
    pipeline = VisionPipeline(config, calibration)
    serial_link = None if args.dry_run else TISerialLink.from_config(config)
    runtime = config["runtime"]
    log_directory = Path(runtime.get("log_directory", "logs"))
    if not log_directory.is_absolute():
        log_directory = config_path.parent / log_directory
    logger = None if args.no_csv else SessionLogger(log_directory)
    video = None
    if args.record_video:
        video_path = Path(args.record_video)
        if not video_path.is_absolute():
            video_path = config_path.parent / video_path
        video_path.parent.mkdir(parents=True, exist_ok=True)
        frame_width = int(config["camera"].get("width", 640))
        frame_height = int(config["camera"].get("height", 480))
        strip_height = int(
            config["tube_reference"].get("strip_size", [640, 120])[1]
        )
        video = DebugVideoWriter(
            str(video_path),
            min(30.0, float(config["camera"].get("fps", 120))),
            (frame_width, frame_height + strip_height),
        )

    stopping = False

    def request_stop(*_: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    protocol_sequence = int(runtime.get("sequence_start", 0)) & 0xFFFFFFFF
    camera_sequence = 0
    previous_timestamp: int | None = None
    frame_period_ms = 0.0
    uart_write_ms = 0.0
    report_started = time.monotonic()
    report_frames = 0
    report_valid = 0
    fps = 0.0
    ti_threshold = float(config["confidence"].get("ti_accept_threshold", 0.70))
    timeouts = 0

    try:
        if serial_link is not None:
            serial_link.open()
            print(
                f"TI UART: {serial_link.port} @ "
                f"{serial_link.baudrate} 8-N-1"
            )
        camera.open()
        print(
            "Vision route: dynamic red(P-)/blue(P+) tube reference "
            "-> replacement-tube Blob -> TI 21-byte V1"
        )
        while not stopping:
            captured = camera.read_after(camera_sequence, timeout=0.25)
            if captured is None:
                timeouts += 1
                if timeouts >= 4:
                    raise RuntimeError("camera delivered no new frame for 1 s")
                continue
            timeouts = 0
            camera_sequence = captured.sequence
            if previous_timestamp is not None:
                period = (
                    (captured.capture_timestamp_us - previous_timestamp)
                    & 0xFFFFFFFF
                ) / 1000.0
                frame_period_ms = (
                    period
                    if frame_period_ms == 0.0
                    else 0.9 * frame_period_ms + 0.1 * period
                )
            previous_timestamp = captured.capture_timestamp_us
            output = pipeline.process(
                captured.image,
                protocol_sequence,
                captured.capture_timestamp_us,
            )
            result = output.result
            tx_position = result.position_mm if result.valid else 0.0
            tx_confidence = result.confidence if result.valid else 0.0
            frame = make_vision_frame(
                protocol_sequence,
                captured.capture_timestamp_us,
                tx_position,
                tx_confidence,
            )
            if serial_link is not None:
                uart_write_ms = serial_link.write(frame)
            else:
                uart_write_ms = 0.0
            protocol_sequence = (protocol_sequence + 1) & 0xFFFFFFFF
            report_frames += 1
            if result.valid and result.confidence >= ti_threshold:
                report_valid += 1

            now = time.monotonic()
            elapsed = now - report_started
            if elapsed >= float(runtime.get("report_interval_s", 1.0)):
                fps = report_frames / elapsed
                valid_ratio = report_valid / max(report_frames, 1)
                print(
                    f"fps={fps:.1f} period={frame_period_ms:.1f}ms "
                    f"process={result.process_time_ms:.1f}ms "
                    f"uart={uart_write_ms:.2f}ms "
                    f"valid={result.valid} state={result.state} "
                    f"position={result.position_mm:+.1f}mm "
                    f"confidence={result.confidence:.2f} "
                    f"TI_accept={result.confidence >= ti_threshold} "
                    f"valid_ratio={valid_ratio:.2%}"
                )
                report_started = now
                report_frames = 0
                report_valid = 0

            if logger is not None:
                logger.write(result, frame_period_ms, uart_write_ms)
            debug = None
            if args.show or video is not None:
                debug = draw_debug(
                    output,
                    fps=fps,
                    frame_period_ms=frame_period_ms,
                    uart_write_ms=uart_write_ms,
                    ti_threshold=ti_threshold,
                )
            if video is not None and debug is not None:
                video.write(debug)
            if args.show and debug is not None:
                cv2.imshow("H-ball Raspberry Pi -> TI", debug)
                if cv2.waitKey(1) & 0xFF in {ord("q"), ord("Q"), 27}:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()
        if serial_link is not None:
            serial_link.close()
        if logger is not None:
            logger.close()
            print(f"CSV: {logger.path}")
        if video is not None:
            video.close()
        if args.show:
            cv2.destroyAllWindows()
        print(f"Stopped at {datetime.now().isoformat(timespec='seconds')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
