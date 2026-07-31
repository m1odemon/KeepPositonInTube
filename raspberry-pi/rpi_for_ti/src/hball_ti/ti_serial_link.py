"""Single-writer UART link to the TI board."""

from __future__ import annotations

import time

import serial


class TISerialLink:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        write_timeout_s: float = 0.05,
    ) -> None:
        if int(baudrate) != 115200:
            raise ValueError("TI vision protocol requires 115200 baud")
        self.port = port
        self.baudrate = int(baudrate)
        self.write_timeout_s = float(write_timeout_s)
        self._serial: serial.Serial | None = None

    @classmethod
    def from_config(cls, config: dict) -> "TISerialLink":
        settings = config["serial"]
        return cls(
            settings.get("port", "/dev/serial0"),
            settings.get("baudrate", 115200),
            settings.get("write_timeout_s", 0.05),
        )

    def open(self) -> None:
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=self.write_timeout_s,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

    def write(self, frame: bytes) -> float:
        if self._serial is None:
            raise RuntimeError("serial link is not open")
        started = time.perf_counter()
        written = self._serial.write(frame)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if written != len(frame):
            raise RuntimeError(
                f"partial UART write: {written}/{len(frame)} bytes"
            )
        return elapsed_ms

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None
