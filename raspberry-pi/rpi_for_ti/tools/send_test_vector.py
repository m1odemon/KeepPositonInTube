"""Send the documented TI protocol test vector without opening a camera."""

from __future__ import annotations

import argparse
import time

from hball_ti.ti_protocol import make_vision_frame
from hball_ti.ti_serial_link import TISerialLink


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--rate", type=float, default=50.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.repeat < 1:
        raise ValueError("--repeat must be at least 1")
    if args.rate <= 0:
        raise ValueError("--rate must be positive")
    link = TISerialLink(args.port)
    link.open()
    try:
        for index in range(args.repeat):
            sequence = 1 + index
            timestamp = 1_000_000 + int(index * 1_000_000 / args.rate)
            frame = make_vision_frame(sequence, timestamp, 0.0, 1.0)
            elapsed_ms = link.write(frame)
            print(
                f"{sequence}: {frame.hex(' ').upper()} "
                f"write={elapsed_ms:.3f}ms"
            )
            if index + 1 < args.repeat:
                time.sleep(1.0 / args.rate)
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
