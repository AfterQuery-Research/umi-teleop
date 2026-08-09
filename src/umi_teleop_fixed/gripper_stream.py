from __future__ import annotations

import socket
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

import cv2
import tyro

from .gripper_protocol import GripperPacket, GripperSample, encode_packet
from .gripper_vision import (
    FisheyeIntrinsics,
    create_aruco_detector,
    detect_gripper_width,
    load_fisheye_intrinsics,
)


@dataclass(frozen=True)
class Args:
    """Stream raw metric UMI gripper widths from two Pi cameras over UDP."""

    target_host: str
    """Hostname or IP address of the robot computer."""
    left_camera: str
    """Stable /dev/v4l/by-path path for the physical left UMI camera."""
    left_intrinsics: Path
    """Fisheye .npz calibration for the left camera."""
    right_camera: str
    """Stable /dev/v4l/by-path path for the physical right UMI camera."""
    right_intrinsics: Path
    """Fisheye .npz calibration for the right camera."""
    target_port: int = 9130
    left_tag_ids: tuple[int, int] = (2, 3)
    right_tag_ids: tuple[int, int] = (0, 1)
    image_width: int = 1600
    image_height: int = 1200
    camera_fps: float = 30.0
    send_rate_hz: float = 30.0
    roi_top_fraction: float = 0.6
    tag_plane_offset_mm: float = 61.9
    tag_front_offset_mm: float = 37.1
    exposure: float = 100.0
    status_interval_seconds: float = 2.0
    startup_timeout_seconds: float = 5.0
    duration_seconds: float | None = None
    sender_name: str = socket.gethostname()


@dataclass(frozen=True)
class CameraStats:
    captured: int
    valid: int


class SampleState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: GripperSample | None = None
        self._captured = 0
        self._valid = 0
        self._fatal_error: str | None = None

    def publish(self, sample: GripperSample) -> None:
        with self._lock:
            self._latest = sample
            self._captured += 1
            self._valid += int(sample.valid)

    def fail(self, message: str) -> None:
        with self._lock:
            self._fatal_error = message
            self._latest = GripperSample.invalid(error=message)

    def snapshot(self) -> tuple[GripperSample | None, CameraStats, str | None]:
        with self._lock:
            return (
                self._latest,
                CameraStats(self._captured, self._valid),
                self._fatal_error,
            )


class CameraWorker:
    def __init__(
        self,
        *,
        side: str,
        device: str,
        tag_ids: tuple[int, int],
        intrinsics: FisheyeIntrinsics,
        args: Args,
    ) -> None:
        self.side = side
        self.device = device
        self.tag_ids = tag_ids
        self.intrinsics = intrinsics
        self.args = args
        self.state = SampleState()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"{side}-camera", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: float = 2.0) -> None:
        self._thread.join(timeout)

    def _open_camera(self) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"could not open {self.device}")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.args.image_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.args.image_height)
        capture.set(cv2.CAP_PROP_FPS, self.args.camera_fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
        capture.set(cv2.CAP_PROP_EXPOSURE, self.args.exposure)
        actual_size = (
            int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        expected_size = self.args.image_width, self.args.image_height
        if actual_size != expected_size:
            capture.release()
            raise RuntimeError(
                f"{self.device} selected {actual_size[0]}x{actual_size[1]}, "
                f"expected {expected_size[0]}x{expected_size[1]}"
            )
        return capture

    def _run(self) -> None:
        capture: cv2.VideoCapture | None = None
        try:
            capture = self._open_camera()
            detector = create_aruco_detector()
            while not self._stop.is_set():
                ok, frame = capture.read()
                captured_ns = time.monotonic_ns()
                if not ok or frame is None:
                    self.state.publish(
                        GripperSample.invalid(
                            captured_monotonic_ns=captured_ns,
                            error="camera read failed",
                        )
                    )
                    continue
                measurement = detect_gripper_width(
                    frame,
                    intrinsics=self.intrinsics,
                    tag_ids=self.tag_ids,
                    detector=detector,
                    roi_top_fraction=self.args.roi_top_fraction,
                    tag_plane_offset_mm=self.args.tag_plane_offset_mm,
                    tag_front_offset_mm=self.args.tag_front_offset_mm,
                )
                self.state.publish(
                    GripperSample(
                        width_mm=measurement.width_mm,
                        valid=measurement.valid,
                        visible_tag_ids=measurement.visible_tag_ids,
                        captured_monotonic_ns=captured_ns,
                    )
                )
        except Exception as exc:
            self.state.fail(f"{self.side} camera failed: {exc}")
        finally:
            if capture is not None:
                capture.release()


def validate_args(args: Args) -> None:
    if not args.target_host:
        raise ValueError("target_host must not be empty")
    if not 1 <= args.target_port <= 65535:
        raise ValueError("target_port must be between 1 and 65535")
    if args.left_camera == args.right_camera:
        raise ValueError("left_camera and right_camera must be different devices")
    if len(set(args.left_tag_ids)) != 2 or len(set(args.right_tag_ids)) != 2:
        raise ValueError("each side requires two distinct tag IDs")
    if set(args.left_tag_ids) & set(args.right_tag_ids):
        raise ValueError("left and right tag IDs must not overlap")
    if args.image_width <= 0 or args.image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if args.camera_fps <= 0 or args.send_rate_hz <= 0:
        raise ValueError("camera_fps and send_rate_hz must be positive")
    if not 0.0 <= args.roi_top_fraction < 1.0:
        raise ValueError("roi_top_fraction must be in [0, 1)")
    if args.tag_plane_offset_mm <= 0 or args.tag_front_offset_mm < 0:
        raise ValueError("tag geometry must be non-negative with a positive plane offset")
    if args.status_interval_seconds <= 0 or args.startup_timeout_seconds <= 0:
        raise ValueError("status and startup intervals must be positive")
    if args.duration_seconds is not None and args.duration_seconds <= 0:
        raise ValueError("duration_seconds must be positive when provided")
    for path in (args.left_intrinsics, args.right_intrinsics):
        if not path.is_file():
            raise ValueError(f"intrinsics file does not exist: {path}")


def _format_sample(sample: GripperSample | None) -> str:
    if sample is None:
        return "starting"
    if sample.valid and sample.width_mm is not None:
        return f"{sample.width_mm:.2f}mm tags={list(sample.visible_tag_ids)}"
    return f"invalid tags={list(sample.visible_tag_ids)}"


def _wait_for_cameras(workers: tuple[CameraWorker, CameraWorker], timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshots = [worker.state.snapshot() for worker in workers]
        fatal_errors = [fatal for _, _, fatal in snapshots if fatal is not None]
        if fatal_errors:
            raise RuntimeError("; ".join(fatal_errors))
        if all(sample is not None for sample, _, _ in snapshots):
            return
        time.sleep(0.02)
    raise RuntimeError("timed out waiting for both cameras")


def run(args: Args) -> None:
    validate_args(args)
    left_worker = CameraWorker(
        side="left",
        device=args.left_camera,
        tag_ids=args.left_tag_ids,
        intrinsics=load_fisheye_intrinsics(args.left_intrinsics),
        args=args,
    )
    right_worker = CameraWorker(
        side="right",
        device=args.right_camera,
        tag_ids=args.right_tag_ids,
        intrinsics=load_fisheye_intrinsics(args.right_intrinsics),
        args=args,
    )
    workers = left_worker, right_worker
    for worker in workers:
        worker.start()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    stream_id = uuid.uuid4().hex
    started = time.monotonic()
    next_send = started
    next_status = started + args.status_interval_seconds
    try:
        _wait_for_cameras(workers, args.startup_timeout_seconds)
        print(
            f"Streaming raw gripper widths to {args.target_host}:{args.target_port} "
            f"at {args.send_rate_hz:g} Hz. Press Ctrl-C to stop.",
            flush=True,
        )
        while args.duration_seconds is None or time.monotonic() - started < args.duration_seconds:
            now_ns = time.monotonic_ns()
            left, left_stats, left_fatal = left_worker.state.snapshot()
            right, right_stats, right_fatal = right_worker.state.snapshot()
            fatal_errors = [error for error in (left_fatal, right_fatal) if error is not None]
            if fatal_errors:
                raise RuntimeError("; ".join(fatal_errors))
            assert left is not None and right is not None
            packet = GripperPacket(
                sequence=sequence,
                sender=args.sender_name,
                stream_id=stream_id,
                sent_monotonic_ns=now_ns,
                left=left,
                right=right,
            )
            udp.sendto(encode_packet(packet), (args.target_host, args.target_port))
            sequence += 1

            now = time.monotonic()
            if now >= next_status:
                left_rate = left_stats.valid / max(now - started, 1e-6)
                right_rate = right_stats.valid / max(now - started, 1e-6)
                print(
                    f"left={_format_sample(left)} valid_rate={left_rate:.1f}Hz  "
                    f"right={_format_sample(right)} valid_rate={right_rate:.1f}Hz  "
                    f"packets={sequence}",
                    flush=True,
                )
                next_status = now + args.status_interval_seconds

            next_send += 1.0 / args.send_rate_hz
            time.sleep(max(0.0, next_send - time.monotonic()))
    except KeyboardInterrupt:
        print("Stopping gripper stream.", flush=True)
    finally:
        udp.close()
        for worker in workers:
            worker.stop()
        for worker in workers:
            worker.join()


def main() -> None:
    run(tyro.cli(Args))


if __name__ == "__main__":
    main()
