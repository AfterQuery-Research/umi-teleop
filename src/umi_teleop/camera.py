"""Capture + decode a local USB camera as RGB ``Frame``s (Pi-side recording, the inversion).

Host: the Pi, when it records locally instead of streaming JPEG to the box. Drives GStreamer
directly via PyGObject (``v4l2src -> jpegdec -> videoconvert -> RGB appsink``) and pulls decoded
RGB frames -- deliberately NOT cv2.VideoCapture: on the Pi, lerobot installs
opencv-python-headless (no GStreamer) which would shadow the system's GStreamer-enabled cv2, so a
cv2-based capture would break once the recorder deps are installed. Using gi (like the sender's
GstJpegCapture) keeps capture independent of whichever cv2 is on the path. A background thread
grabs frames, stamps them with the local clock, and buffers them in a bounded deque; mirrors
``TcpCameraSource``'s ``Camera`` contract but reads the device directly -- no network, no transport
bottleneck. The appsink emits RGB, matching the rest of the pipeline (``decode_jpeg`` yields RGB).
"""

from __future__ import annotations

import os
import subprocess
import threading
import time
from collections import deque
from collections.abc import Callable

import numpy as np

from umi_teleop.types import Frame


def build_pipeline(device: str, width: int, height: int, fps: int) -> str:
    """GStreamer pipeline: MJPEG off the device, decoded to an RGB appsink."""
    return (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
        "jpegdec ! videoconvert ! video/x-raw,format=RGB ! "
        "appsink name=sink drop=true max-buffers=2 sync=false"
    )


class GstRgbCapture:
    """Pull decoded RGB frames from a local camera via a GStreamer appsink (PyGObject).

    ``read()`` returns ``(ok, rgb_hwc_uint8)`` (or ``(False, None)`` at EOS), mirroring
    cv2.VideoCapture so it slots into ``LocalGstCamera`` behind the ``capture_factory`` seam.
    """

    def __init__(self, device: str, width: int, height: int, fps: int) -> None:
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        self._Gst = Gst
        self.width = width
        self.height = height
        self._pipeline = Gst.parse_launch(build_pipeline(device, width, height, fps))
        self._sink = self._pipeline.get_by_name("sink")
        if self._pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise SystemExit(f"could not start local camera pipeline for {device}")

    def read(self) -> tuple[bool, np.ndarray | None]:
        sample = self._sink.emit("pull-sample")  # blocks for the next frame; None at EOS/error
        if sample is None:
            return False, None
        buf = sample.get_buffer()
        ok, minfo = buf.map(self._Gst.MapFlags.READ)
        if not ok:
            return False, None
        try:
            arr = np.frombuffer(bytes(minfo.data), np.uint8)  # copy out before unmap
        finally:
            buf.unmap(minfo)
        h, w = self.height, self.width
        if arr.size == h * w * 3:
            return True, arr.reshape(h, w, 3)
        stride = arr.size // h  # RGB rows may be padded to a 4-byte boundary
        return True, arr.reshape(h, stride)[:, : w * 3].reshape(h, w, 3)

    def release(self) -> None:
        self._pipeline.set_state(self._Gst.State.NULL)


def set_manual_exposure(device: str, exposure: int) -> None:
    """Pin manual exposure via v4l2-ctl (mirrors umi_sender.set_manual_exposure). Best-effort."""
    dev = os.path.realpath(device)  # v4l2-ctl wants the real /dev/videoN, not a by-path symlink
    try:
        subprocess.run(
            [
                "v4l2-ctl",
                "-d",
                dev,
                "--set-ctrl",
                "auto_exposure=1",
                "--set-ctrl",
                f"exposure_time_absolute={exposure}",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        )
        print(f"manual exposure: exposure_time_absolute={exposure} on {dev}", flush=True)
    except (subprocess.SubprocessError, FileNotFoundError, OSError) as e:
        print(f"WARN: could not set manual exposure on {dev}: {e}", flush=True)


class LocalGstCamera:
    """Local USB camera as a ``Camera``: a background thread grabs RGB frames into a deque.

    ``capture_factory`` (a no-arg callable returning an object with ``read() -> (ok, rgb)`` and
    ``release()``) is injectable so the grab/deque logic is testable without a real camera; it
    defaults to :class:`GstRgbCapture`.
    """

    def __init__(
        self,
        device: str,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        exposure: int = 0,
        # small on purpose: the App drains continuously, and 256 (the box default) x a 1024x768
        # frame x2 cams is ~1.2GB of buffers -> OOM on the 4GB Pi. 16 caps it (~75MB) and drops
        # excess frames gracefully instead of ballooning memory.
        queue_size: int = 16,
        capture_factory: Callable[[], object] | None = None,
    ):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.exposure = exposure
        self.frames: deque[Frame] = deque(maxlen=queue_size)
        self._open = capture_factory or (lambda: GstRgbCapture(device, width, height, fps))
        self._cap: object | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._cap is not None:
            return
        self._cap = self._open()
        if self.exposure > 0:  # after open so v4l2src can't reset it (see umi_sender)
            set_manual_exposure(self.device, self.exposure)
        self._stop.clear()
        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

    def _grab_loop(self) -> None:
        while not self._stop.is_set():
            ok, rgb = self._cap.read()  # type: ignore[union-attr]
            ts = (
                time.time()
            )  # stamp at grab, local clock (shared with the pose receiver's arrivals)
            if not ok or rgb is None:
                time.sleep(0.002)  # transient glitch or EOS: back off, don't busy-spin
                continue
            with self._lock:
                self.frames.append(Frame(timestamp=ts, image=rgb))

    def read(self) -> list[Frame]:
        with self._lock:
            frames = list(self.frames)
            self.frames.clear()
        return frames

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._cap is not None:
            self._cap.release()  # type: ignore[union-attr]
            self._cap = None
        with self._lock:
            self.frames.clear()
