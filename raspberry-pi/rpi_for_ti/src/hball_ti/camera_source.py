"""Low-latency USB camera that exposes only the newest frame."""

from __future__ import annotations

import threading
import time
import subprocess
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True, slots=True)
class CapturedFrame:
    image: np.ndarray
    sequence: int
    capture_timestamp_us: int


def monotonic_timestamp_us() -> int:
    return (time.monotonic_ns() // 1_000) & 0xFFFFFFFF


def has_gstreamer() -> bool:
    return any(
        "GStreamer:" in line and "YES" in line.upper()
        for line in cv2.getBuildInformation().splitlines()
    )


def gstreamer_pipeline(
    device: str,
    width: int,
    height: int,
    fps: int,
    pixel_format: str,
) -> str:
    if pixel_format.upper() == "MJPG":
        rates = {
            (640, 480, 30): "151/5",
            (640, 480, 60): "121/2",
            (640, 480, 90): "90/1",
            (640, 480, 120): "61612/513",
        }
        rate = rates.get((width, height, fps), f"{fps}/1")
        caps = (
            f"image/jpeg,width={width},height={height},framerate={rate}"
        )
        conversion = (
            "jpegdec ! videoconvert ! video/x-raw,format=BGR"
        )
    elif pixel_format.upper() == "YUYV":
        caps = (
            f"video/x-raw,format=YUY2,width={width},height={height},"
            f"framerate={fps}/1"
        )
        conversion = "videoconvert ! video/x-raw,format=BGR"
    else:
        raise ValueError("pixel_format must be MJPG or YUYV")
    return (
        f"v4l2src device={device} io-mode=2 ! {caps} ! {conversion} ! "
        "appsink max-buffers=1 drop=true sync=false"
    )


class LatestFrameCamera:
    def __init__(
        self,
        *,
        device: str,
        width: int,
        height: int,
        fps: int,
        pixel_format: str,
        backend: str,
        controls: dict | None = None,
    ) -> None:
        self.device = device
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.pixel_format = pixel_format.upper()
        self.backend = backend.lower()
        self.controls = dict(controls or {})
        self._capture: cv2.VideoCapture | None = None
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._stopping = False
        self._latest: CapturedFrame | None = None
        self._sequence = 0
        self.failures = 0

    @classmethod
    def from_config(cls, config: dict) -> "LatestFrameCamera":
        camera = config["camera"]
        return cls(
            device=camera.get("device", "/dev/video0"),
            width=camera.get("width", 640),
            height=camera.get("height", 480),
            fps=camera.get("fps", 120),
            pixel_format=camera.get("pixel_format", "MJPG"),
            backend=camera.get("backend", "gstreamer"),
            controls=camera.get("controls", {}),
        )

    def _apply_controls(self) -> None:
        if not self.controls:
            return
        values = ",".join(
            f"{name}={int(value)}"
            for name, value in self.controls.items()
        )
        try:
            subprocess.run(
                [
                    "v4l2-ctl",
                    "-d",
                    self.device,
                    f"--set-ctrl={values}",
                ],
                check=True,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(
                "camera controls were configured but v4l2-ctl is missing"
            ) from exc
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                "failed to apply camera controls; inspect "
                "`v4l2-ctl --list-ctrls-menus`"
            ) from exc

    def open(self) -> None:
        self._apply_controls()
        if self.backend == "gstreamer":
            if not has_gstreamer():
                raise RuntimeError("OpenCV was built without GStreamer")
            pipeline = gstreamer_pipeline(
                self.device,
                self.width,
                self.height,
                self.fps,
                self.pixel_format,
            )
            print(f"GStreamer pipeline:\n{pipeline}")
            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        else:
            index = int(self.device.removeprefix("/dev/video"))
            capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
            capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*self.pixel_format),
            )
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open camera {self.device}")
        self._capture = capture
        self._stopping = False
        self._thread = threading.Thread(
            target=self._loop,
            name="ti-camera-latest-frame",
            daemon=True,
        )
        self._thread.start()

    def _loop(self) -> None:
        assert self._capture is not None
        while not self._stopping:
            ok, frame = self._capture.read()
            timestamp = monotonic_timestamp_us()
            if not ok or frame is None:
                self.failures += 1
                continue
            with self._condition:
                self._sequence += 1
                self._latest = CapturedFrame(
                    frame,
                    self._sequence,
                    timestamp,
                )
                self._condition.notify_all()

    def read_after(
        self,
        sequence: int,
        timeout: float = 0.25,
    ) -> CapturedFrame | None:
        deadline = time.monotonic() + timeout
        with self._condition:
            while (
                self._latest is None
                or self._latest.sequence <= sequence
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return self._latest

    def close(self) -> None:
        self._stopping = True
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._capture is not None:
            self._capture.release()
        with self._condition:
            self._condition.notify_all()

    def __enter__(self) -> "LatestFrameCamera":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
