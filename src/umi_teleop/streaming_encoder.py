"""Real-time LeRobot encoder optimization for the pinned dataset dependency."""

from __future__ import annotations

import contextlib
import logging
import queue
from fractions import Fraction
from pathlib import Path
from threading import Lock
from typing import Any

import numpy as np

_PATCH_MARKER = "_umi_deferred_rgb_stats"
_AV_LOG_LOCK = Lock()
_AV_LOG_USERS = 0
_AV_LOG_PREVIOUS_LEVEL: int | None = None


@contextlib.contextmanager
def _suppress_av_conversion_warnings():
    """Keep PyAV's process-global log level quiet across concurrent stats workers."""
    import av

    global _AV_LOG_PREVIOUS_LEVEL, _AV_LOG_USERS
    with _AV_LOG_LOCK:
        if _AV_LOG_USERS == 0:
            _AV_LOG_PREVIOUS_LEVEL = av.logging.get_level()
            av.logging.set_level(av.logging.ERROR)
        _AV_LOG_USERS += 1
    try:
        yield
    finally:
        with _AV_LOG_LOCK:
            _AV_LOG_USERS -= 1
            if _AV_LOG_USERS == 0:
                av.logging.set_level(_AV_LOG_PREVIOUS_LEVEL)
                _AV_LOG_PREVIOUS_LEVEL = None


def install_streaming_encoder_optimization() -> None:
    """Keep RGB conversion and statistics from starving live H.264 encoding.

    The project pins the exact LeRobot revision whose private encoder thread is
    adapted here. Patching its worker method lets the public LeRobot dataset API
    and on-disk format remain unchanged while avoiding a separate fork install.
    """
    from lerobot.datasets import video_utils

    thread_type = video_utils._CameraEncoderThread
    if getattr(thread_type, _PATCH_MARKER, False):
        return
    thread_type.run = _run_streaming_encoder  # type: ignore[method-assign]
    setattr(thread_type, _PATCH_MARKER, True)


def _compute_rgb_video_stats(video_path: Path) -> dict[str, np.ndarray]:
    """Compute statistics from every frame of a completed RGB video."""
    import av
    from lerobot.datasets.compute_stats import RunningQuantileStats, auto_downsample_height_width

    tracker = RunningQuantileStats()
    frame_count = 0
    with _suppress_av_conversion_warnings():
        # ARM FFmpeg can otherwise emit this performance warning once per frame.
        with av.open(str(video_path)) as container:
            for video_frame in container.decode(video=0):
                image = video_frame.to_ndarray(format="rgb24")
                image_chw = auto_downsample_height_width(image.transpose(2, 0, 1))
                channels = image_chw.shape[0]
                tracker.update(image_chw.transpose(1, 2, 0).reshape(-1, channels))
                frame_count += 1
    if frame_count < 2:
        raise RuntimeError(f"encoded video contains only {frame_count} frame(s): {video_path}")
    return tracker.get_statistics()


def _run_streaming_encoder(self: Any) -> None:
    """Encode RGB directly with PyAV and defer RGB statistics until completion."""
    import av
    from lerobot.datasets.compute_stats import RunningQuantileStats, auto_downsample_height_width
    from lerobot.datasets.video_utils import quantize_depth

    container = None
    output_stream = None
    # Preserve LeRobot's existing inline behavior for depth video. The YAM
    # recorder has RGB cameras only, and decoded depth may not preserve its unit.
    stats_tracker = RunningQuantileStats() if self.is_depth else None
    frame_count = 0

    try:
        logging.getLogger("libav").setLevel(av.logging.WARNING)
        while True:
            try:
                frame_data = self.frame_queue.get(timeout=1)
            except queue.Empty:
                if self.stop_event.is_set():
                    break
                continue

            if frame_data is None:
                break

            if isinstance(frame_data, np.ndarray):
                if frame_data.ndim == 3 and frame_data.shape[0] in (1, 3):
                    frame_data = frame_data.transpose(1, 2, 0)
                if not self.is_depth and frame_data.dtype != np.uint8:
                    frame_data = (frame_data * 255).astype(np.uint8)

            if container is None:
                height, width = frame_data.shape[:2]
                Path(self.video_path).parent.mkdir(parents=True, exist_ok=True)
                container = av.open(str(self.video_path), "w")
                output_stream = container.add_stream(
                    self.video_encoder.vcodec,
                    self.fps,
                    options=self.video_encoder.get_codec_options(
                        self.encoder_threads, as_strings=True
                    ),
                )
                output_stream.pix_fmt = self.video_encoder.pix_fmt
                output_stream.width = width
                output_stream.height = height
                output_stream.time_base = Fraction(1, self.fps)

            if self.is_depth:
                video_frame = quantize_depth(
                    frame_data,
                    depth_min=self.video_encoder.depth_min,
                    depth_max=self.video_encoder.depth_max,
                    shift=self.video_encoder.shift,
                    use_log=self.video_encoder.use_log,
                    video_backend=self.video_encoder.video_backend,
                )
            else:
                video_frame = av.VideoFrame.from_ndarray(frame_data, format="rgb24")
            video_frame.pts = frame_count
            video_frame.time_base = Fraction(1, self.fps)
            packet = output_stream.encode(video_frame)
            if packet:
                container.mux(packet)

            if stats_tracker is not None:
                image_chw = auto_downsample_height_width(frame_data.transpose(2, 0, 1))
                channels = image_chw.shape[0]
                stats_tracker.update(image_chw.transpose(1, 2, 0).reshape(-1, channels))
            frame_count += 1

        if output_stream is not None:
            packet = output_stream.encode()
            if packet:
                container.mux(packet)
        if container is not None:
            container.close()

        if frame_count >= 2:
            stats = (
                stats_tracker.get_statistics()
                if stats_tracker is not None
                else _compute_rgb_video_stats(self.video_path)
            )
            self.result_queue.put(("ok", stats))
        else:
            self.result_queue.put(("ok", None))
    except Exception as exc:
        if container is not None:
            with contextlib.suppress(Exception):
                container.close()
        self.result_queue.put(("error", str(exc)))
