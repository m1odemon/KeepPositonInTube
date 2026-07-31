"""Read-only dependency, configuration and protocol self-check."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy
import serial
import yaml

from hball_ti.camera_source import gstreamer_pipeline, has_gstreamer
from hball_ti.ti_protocol import make_vision_frame


EXPECTED_VECTOR = (
    "A5 5A 01 01 00 00 00 40 42 0F 00 "
    "00 00 00 00 00 00 80 3F 99 CC"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--calibration", default="calibration.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = yaml.safe_load(
        Path(args.config).read_text(encoding="utf-8")
    )
    calibration = json.loads(
        Path(args.calibration).read_text(encoding="utf-8")
    )
    camera = config["camera"]
    frame = make_vision_frame(1, 1_000_000, 0.0, 1.0)
    vector = frame.hex(" ").upper()
    if vector != EXPECTED_VECTOR:
        raise RuntimeError(f"protocol vector mismatch: {vector}")
    print(f"OpenCV: {cv2.__version__}")
    print(f"NumPy: {numpy.__version__}")
    print(f"pyserial: {serial.VERSION}")
    print(f"PyYAML: {yaml.__version__}")
    print(f"OpenCV GStreamer: {'YES' if has_gstreamer() else 'NO'}")
    print(
        "Calibration: "
        f"{calibration['position_min_mm']:+.1f} .. "
        f"{calibration['position_max_mm']:+.1f} mm, "
        f"zero_u={calibration['zero_u']:.4f}"
    )
    print(
        gstreamer_pipeline(
            camera["device"],
            int(camera["width"]),
            int(camera["height"]),
            int(camera["fps"]),
            camera["pixel_format"],
        )
    )
    print(f"Protocol vector OK ({len(frame)} bytes): {vector}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
