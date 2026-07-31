"""Interactive capture and dynamic tube-ROI setup for a changed camera pose."""

from __future__ import annotations

import argparse
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from hball_ti.camera_source import LatestFrameCamera
from hball_ti.config import load_json, load_yaml
from hball_ti.tube_reference import TubeReference, TubeReferenceDetector


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview, adjust and save the per-frame dynamic tube ROI",
    )
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--calibration", default="calibration.json")
    parser.add_argument(
        "--output-dir",
        default="calibration_images",
    )
    return parser.parse_args()


def point(value: tuple[float, float]) -> tuple[int, int]:
    return int(round(value[0])), int(round(value[1]))


def roi_polygon(reference: TubeReference) -> np.ndarray | None:
    if reference.strip is None:
        return None
    width = reference.strip.shape[1]
    height = reference.strip.shape[0]
    values = (
        reference.frame_point_from_strip(0, 0),
        reference.frame_point_from_strip(width - 1, 0),
        reference.frame_point_from_strip(width - 1, height - 1),
        reference.frame_point_from_strip(0, height - 1),
    )
    if any(value is None for value in values):
        return None
    return np.asarray(
        [point(value) for value in values if value is not None],
        dtype=np.int32,
    )


def draw_setup(
    frame: np.ndarray,
    reference: TubeReference,
    half_width: float,
    axis_samples: deque[float],
) -> np.ndarray:
    display = frame.copy()
    polygon = roi_polygon(reference)
    if polygon is not None:
        cv2.polylines(
            display,
            [polygon],
            True,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    if reference.p_negative is not None:
        cv2.circle(display, point(reference.p_negative), 9, (0, 0, 255), 2)
        cv2.putText(
            display,
            "P- RED",
            point(reference.p_negative),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
    if reference.p_positive is not None:
        cv2.circle(display, point(reference.p_positive), 9, (255, 0, 0), 2)
        cv2.putText(
            display,
            "P+ BLUE",
            point(reference.p_positive),
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
            display,
            point(reference.p_negative),
            point(reference.p_positive),
            (0, 200, 255),
            2,
            cv2.LINE_AA,
        )
    median_axis = (
        float(np.median(np.asarray(axis_samples)))
        if axis_samples
        else 0.0
    )
    status = "VALID" if reference.valid else "NO RED/BLUE REFERENCE"
    cv2.rectangle(display, (0, 0), (display.shape[1], 82), (0, 0, 0), -1)
    cv2.putText(
        display,
        (
            f"{status} half_width={half_width:.0f}px "
            f"axis={reference.axis_length_px:.1f}px "
            f"median={median_axis:.1f}px"
        ),
        (8, 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display,
        "[ / ] ROI width   C capture   P save config+capture   Q quit",
        (8, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.47,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )
    strip = (
        reference.strip
        if reference.strip is not None
        else np.zeros((120, 640, 3), dtype=np.uint8)
    )
    if strip.shape[1] != display.shape[1]:
        strip = cv2.resize(strip, (display.shape[1], strip.shape[0]))
    return np.vstack((display, strip))


def save_capture(
    output_root: Path,
    frame: np.ndarray,
    reference: TubeReference,
    half_width: float,
    median_axis: float,
) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    session = output_root / timestamp
    session.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(session / "camera_original.jpg"), frame)
    if reference.strip is not None:
        cv2.imwrite(str(session / "dynamic_roi_strip.jpg"), reference.strip)
    metadata = {
        "captured_at": datetime.now().isoformat(timespec="milliseconds"),
        "tube_half_width_px": half_width,
        "axis_length_px": reference.axis_length_px,
        "median_axis_length_px": median_axis,
        "angle_deg": reference.angle_deg,
        "reference_confidence": reference.confidence,
        "p_negative_px": reference.p_negative,
        "p_positive_px": reference.p_positive,
    }
    (session / "roi_measurement.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return session


def persist_config(
    config_path: Path,
    config: dict,
    half_width: float,
    median_axis: float,
) -> Path:
    if median_axis <= 0:
        raise RuntimeError("no valid tube-axis measurements to save")
    settings = config["tube_reference"]
    settings["tube_half_width_px"] = int(round(half_width))
    settings["expected_axis_length_px"] = int(round(median_axis))
    settings["axis_length_min_px"] = int(round(median_axis * 0.70))
    settings["axis_length_max_px"] = int(round(median_axis * 1.30))
    backup = config_path.with_suffix(".before_roi_setup.yaml")
    if not backup.exists():
        backup.write_text(
            config_path.read_text(encoding="utf-8"),
            encoding="utf-8",
        )
    config_path.write_text(
        yaml.safe_dump(
            config,
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    return backup


def main() -> int:
    args = parse_args()
    config_path = Path(args.config).resolve()
    calibration_path = Path(args.calibration).resolve()
    output_root = Path(args.output_dir)
    if not output_root.is_absolute():
        output_root = config_path.parent / output_root
    config = load_yaml(config_path)
    calibration = load_json(calibration_path)
    zero_u = float(calibration["zero_u"])
    reference_settings = dict(config["tube_reference"])

    # Setup mode deliberately accepts a broad axis range. The saved result
    # restores a tight ±30% runtime range around the new measured median.
    reference_settings["axis_length_min_px"] = 40
    reference_settings["axis_length_max_px"] = 900
    reference_settings["endpoint_jump_gate_px"] = 250
    reference_settings["max_abs_angle_deg"] = 45
    reference_settings["marker_area_min"] = 20
    reference_settings["marker_area_max"] = 20_000
    detector = TubeReferenceDetector(reference_settings, zero_u)
    half_width = float(config["tube_reference"]["tube_half_width_px"])
    detector.half_width = half_width
    camera = LatestFrameCamera.from_config(config)
    axis_samples: deque[float] = deque(maxlen=120)
    camera_sequence = 0

    print("Place matte RED at P- and matte BLUE at P+.")
    print("Use [ and ] to make the yellow ROI narrower/wider.")
    print("Press C to capture, P to save config and capture, Q to quit.")
    try:
        camera.open()
        while True:
            captured = camera.read_after(camera_sequence, timeout=0.5)
            if captured is None:
                raise RuntimeError("camera delivered no new frame for 0.5 s")
            camera_sequence = captured.sequence
            detector.half_width = half_width
            reference = detector.process(captured.image)
            if reference.valid:
                axis_samples.append(reference.axis_length_px)
            display = draw_setup(
                captured.image,
                reference,
                half_width,
                axis_samples,
            )
            cv2.imshow("Dynamic tube ROI setup", display)
            key = cv2.waitKey(1) & 0xFF
            if key in {ord("q"), ord("Q"), 27}:
                break
            if key == ord("["):
                half_width = max(10.0, half_width - 2.0)
            elif key == ord("]"):
                half_width = min(180.0, half_width + 2.0)
            elif key in {ord("c"), ord("C")}:
                if not reference.valid:
                    print("Capture rejected: red/blue reference is invalid.")
                    continue
                median_axis = float(np.median(np.asarray(axis_samples)))
                session = save_capture(
                    output_root,
                    captured.image,
                    reference,
                    half_width,
                    median_axis,
                )
                print(f"Captured: {session}")
            elif key in {ord("p"), ord("P")}:
                if not reference.valid or len(axis_samples) < 20:
                    print(
                        "Save rejected: keep both markers visible for "
                        "at least 20 valid frames."
                    )
                    continue
                median_axis = float(np.median(np.asarray(axis_samples)))
                session = save_capture(
                    output_root,
                    captured.image,
                    reference,
                    half_width,
                    median_axis,
                )
                backup = persist_config(
                    config_path,
                    config,
                    half_width,
                    median_axis,
                )
                print(
                    f"Saved ROI: half_width={half_width:.0f}px, "
                    f"axis={median_axis:.1f}px"
                )
                print(f"Capture: {session}")
                print(f"Config backup: {backup}")
                time.sleep(0.4)
                break
    finally:
        camera.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
