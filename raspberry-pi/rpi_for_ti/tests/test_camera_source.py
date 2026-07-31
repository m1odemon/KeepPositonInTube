from __future__ import annotations

import sys
import unittest
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))


class CameraSourceTests(unittest.TestCase):
    def test_known_camera_mode_uses_exact_fraction(self) -> None:
        from hball_ti.camera_source import gstreamer_pipeline

        pipeline = gstreamer_pipeline(
            "/dev/video0",
            640,
            480,
            120,
            "MJPG",
        )
        self.assertIn("image/jpeg,width=640,height=480", pipeline)
        self.assertIn("framerate=61612/513", pipeline)
        self.assertIn("appsink max-buffers=1 drop=true", pipeline)

    def test_yuyv_uses_raw_yuy2_caps(self) -> None:
        from hball_ti.camera_source import gstreamer_pipeline

        pipeline = gstreamer_pipeline(
            "/dev/video2",
            640,
            480,
            30,
            "YUYV",
        )
        self.assertIn("device=/dev/video2", pipeline)
        self.assertIn("video/x-raw,format=YUY2", pipeline)
        self.assertIn("framerate=30/1", pipeline)


if __name__ == "__main__":
    unittest.main()
