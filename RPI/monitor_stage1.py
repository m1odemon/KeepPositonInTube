"""第一阶段串口监视器：只接收F407遥测，不发送电机控制命令。"""

from __future__ import annotations

import argparse
import time

from hball_protocol import make_telemetry_decoder


STATE_NAMES = {
    0: "DISABLED",
    1: "CALIBRATION_REQUIRED",
    2: "AWAIT_MOTOR_FEEDBACK",
    3: "AWAIT_VISION",
    4: "TRACKING",
    5: "SAFE_LEVEL",
    6: "FAULT",
}

FAULT_NAMES = {
    0: "CALIBRATION_INVALID",
    1: "MOTOR_FEEDBACK_TIMEOUT",
    2: "COMMAND_TIMEOUT",
    3: "MOTOR_ANGLE_LIMIT",
    4: "CAN_TRANSMIT",
    5: "CAN_PERIPHERAL",
    6: "INVALID_COMMAND",
}


def describe_faults(flags: int) -> str:
    names = [
        name for bit, name in FAULT_NAMES.items() if flags & (1 << bit)
    ]
    return "|".join(names) if names else "NONE"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="例如 COM8 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_152_000)
    args = parser.parse_args()

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("请先执行: python -m pip install pyserial") from exc

    decoder = make_telemetry_decoder()
    last_print = 0.0
    with serial.Serial(args.port, args.baud, timeout=0.2) as uart:
        while True:
            for telemetry in decoder.feed(uart.read(256)):
                now = time.monotonic()
                if now - last_print < 0.2:
                    continue
                last_print = now
                print(
                    f"state={STATE_NAMES.get(telemetry.state, telemetry.state)} "
                    f"fault={describe_faults(telemetry.fault_flags)} "
                    f"motor_feedback={'YES' if telemetry.flags & 0x02 else 'NO'} "
                    f"motor_enabled={'YES' if telemetry.flags & 0x04 else 'NO'} "
                    f"angle={telemetry.motor_angle_rad:+.4f}rad "
                    f"speed={telemetry.motor_speed_rpm:+.2f}rpm "
                    f"current={telemetry.motor_current_a:+.3f}A "
                    f"motor_age={telemetry.motor_feedback_age_ms}ms"
                )


if __name__ == "__main__":
    main()

