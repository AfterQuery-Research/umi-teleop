"""LocalGstCamera (Pi-side capture): pipeline string + grab/deque logic.

The real GStreamer/gi capture needs a Pi + camera, so the Camera logic is tested with an injected
fake capture returning synthetic RGB frames. build_pipeline is a pure function; the BGR->RGB and
decode happen inside GStreamer (appsink emits RGB), so the camera itself passes frames through.
"""

from __future__ import annotations

import time

import numpy as np

from umi_teleop.camera import LocalGstCamera, build_pipeline


def test_build_pipeline_has_device_caps_and_rgb_decoder():
    p = build_pipeline("/dev/video2", 640, 480, 30)
    assert "v4l2src device=/dev/video2" in p
    assert "image/jpeg,width=640,height=480,framerate=30/1" in p
    assert "jpegdec" in p and "format=RGB" in p and "appsink" in p


class _FakeCap:
    """GstRgbCapture stand-in: always returns the same RGB frame."""

    def __init__(self, rgb):
        self.rgb = rgb
        self.released = False

    def read(self):
        return True, self.rgb.copy()

    def release(self):
        self.released = True


def _rgb_frame():
    f = np.zeros((4, 4, 3), dtype=np.uint8)
    f[..., 0], f[..., 1], f[..., 2] = 200, 20, 10
    return f


def _drain(cam, want=1, timeout=2.0):
    got = []
    t0 = time.time()
    while len(got) < want and time.time() - t0 < timeout:
        got.extend(cam.read())
        time.sleep(0.01)
    return got


def test_local_camera_emits_frames_unchanged():
    rgb = _rgb_frame()
    cam = LocalGstCamera("/dev/fake", exposure=0, capture_factory=lambda: _FakeCap(rgb))
    cam.start()
    try:
        frames = _drain(cam, want=1)
        assert frames, "no frames grabbed"
        assert np.array_equal(frames[0].image, rgb)  # appsink emits RGB; camera passes it through
        assert frames[0].timestamp > 0
        cam.read()  # clears the deque (fake is free-running, refills immediately)
    finally:
        cam.stop()


def test_local_camera_start_idempotent_and_stop_releases():
    opens = []

    def factory():
        c = _FakeCap(_rgb_frame())
        opens.append(c)
        return c

    cam = LocalGstCamera("/dev/fake", capture_factory=factory)
    cam.start()
    cam.start()  # second start is a no-op (already open)
    assert len(opens) == 1
    _drain(cam, want=1)
    cam.stop()
    assert cam.read() == []
    assert opens[0].released
