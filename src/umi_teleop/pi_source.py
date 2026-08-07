"""Raspberry Pi camera, ArUco, and GPIO publisher for native teleoperation."""

from __future__ import annotations

import logging
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import tyro

from umi_teleop.buttons import GpioButtons
from umi_teleop.pi_protocol import PI_PROTOCOL_VERSION, PiEventState, encode_pi_packet
from umi_teleop.protocol import ARM_NAMES, WidthWorker

DATA_DIR = Path(__file__).resolve().parent / "data"
UMI1_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0"
UMI2_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:2:1.0-video-index0"


@dataclass(frozen=True)
class NetworkArgs:
    """Pi-to-controller UDP transport."""

    target_host: str = "aq-yam.local"
    """aq-yam hostname or IPv4 address receiving sensor telemetry."""

    target_port: int = 9139
    """UDP destination port on aq-yam."""

    publish_hz: float = 30.0
    """Heartbeat and sensor packet frequency."""


@dataclass(frozen=True)
class CameraArgs:
    """One Pi-mounted gripper camera and its ArUco geometry."""

    device: str
    """Stable V4L2 device path, preferably under /dev/v4l/by-path or by-id."""

    calibration: Path
    """Fisheye calibration NPZ file on this Raspberry Pi."""

    tag_ids: tuple[int, int]
    """Two ArUco tag IDs attached to this gripper."""

    width: int = 800
    """Requested capture width in pixels."""

    height: int = 600
    """Requested capture height in pixels."""

    fps: int = 30
    """Requested camera capture rate in frames per second."""

    exposure: int = 100
    """Manual V4L2 exposure_time_absolute value."""

    detect_hz: float = 15.0
    """Maximum ArUco detection frequency for this camera."""

    roi_top_frac: float = 0.0
    """Fraction of the image height excluded above the detection region."""


def _left_camera() -> CameraArgs:
    return CameraArgs(UMI2_DEVICE, DATA_DIR / "umi2_calib.npz", (2, 3))


def _right_camera() -> CameraArgs:
    return CameraArgs(UMI1_DEVICE, DATA_DIR / "umi1_calib.npz", (0, 1))


@dataclass(frozen=True)
class GpioArgs:
    """Two active-low physical buttons wired to Raspberry Pi GPIO."""

    record_pin: int = 17
    """Tap to start/stop; hold to cancel and discard the active episode."""

    zero_quit_pin: int = 27
    """Tap to zero; hold to quit safely."""

    hold_s: float = 1.5
    """Seconds a button must remain pressed to emit its hold action."""

    bounce_s: float = 0.05
    """gpiozero hardware-contact debounce interval in seconds."""

    toggle_debounce_s: float = 0.4
    """Minimum interval between accepted RECORD tap events."""


@dataclass(frozen=True)
class Args:
    """Publish Raspberry Pi gripper measurements and GPIO events to aq-yam."""

    network: NetworkArgs = field(default_factory=NetworkArgs)
    left: CameraArgs = field(default_factory=_left_camera)
    right: CameraArgs = field(default_factory=_right_camera)
    gpio: GpioArgs = field(default_factory=GpioArgs)
    camera_timeout_s: float = 0.75
    """Maximum camera-frame age before camera_ok becomes false."""

    width_timeout_s: float = 0.75
    """Maximum ArUco width-measurement age before telemetry becomes unhealthy."""

    preflight_timeout_s: float = 20.0
    """Seconds allowed for both cameras and tag pairs to become healthy."""

    quit_burst_s: float = 0.75
    """Seconds to repeat a QUIT counter before the Pi publisher exits."""
    preflight_only: bool = False
    """Validate both cameras, tag pairs, and GPIO without remaining online."""

    simulate: bool = False
    """Publish deterministic fake widths without opening cameras or GPIO."""

    max_packets: int | None = None
    """Stop after this many packets; intended for automated tests."""

    quiet: bool = True
    """Show only GPIO state changes, warnings, and errors; pass --no-quiet for diagnostics."""


class _SimulatedWorker:
    def __init__(self, width_mm: float) -> None:
        self.width_mm = width_mm

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def snapshot(self, _now: float, *, camera_timeout_s: float) -> dict[str, Any]:
        del camera_timeout_s
        return {
            "width_mm": self.width_mm,
            "age_s": 0.0,
            "tags_visible": True,
            "camera_ok": True,
            "frame_age_s": 0.0,
            "frames_seen": 1,
            "detections": 1,
            "misses": 0,
            "errors": 0,
            "last_error": None,
        }


class _NoButtons:
    def poll(self) -> None:
        return None

    def close(self) -> None:
        pass


def validate_args(args: Args) -> None:
    """Reject invalid or ambiguous hardware configuration before opening devices."""
    errors: list[str] = []
    if not args.network.target_host.strip():
        errors.append("--network.target-host must not be empty")
    if not 1 <= args.network.target_port <= 65_535:
        errors.append("--network.target-port must be between 1 and 65535")
    if not math.isfinite(args.network.publish_hz) or not 1.0 <= args.network.publish_hz <= 120.0:
        errors.append("--network.publish-hz must be between 1 and 120")
    if args.left.device == args.right.device:
        errors.append("left and right camera devices must be different")
    all_tags = (*args.left.tag_ids, *args.right.tag_ids)
    if any(tag < 0 for tag in all_tags):
        errors.append("ArUco tag IDs must be non-negative")
    if len(set(all_tags)) != len(all_tags):
        errors.append("left and right ArUco tag IDs must be distinct")
    for name, camera in (("left", args.left), ("right", args.right)):
        if camera.tag_ids[0] == camera.tag_ids[1]:
            errors.append(f"{name} ArUco tag IDs must be different")
        if camera.width <= 0 or camera.height <= 0 or camera.fps <= 0:
            errors.append(f"{name} camera width, height, and FPS must be positive")
        if not math.isfinite(camera.detect_hz) or camera.detect_hz <= 0:
            errors.append(f"{name} camera detection frequency must be positive")
        if not 0.0 <= camera.roi_top_frac <= 0.95:
            errors.append(f"{name} camera ROI top fraction must be between 0 and 0.95")
        if not args.simulate:
            if not Path(camera.device).exists():
                errors.append(f"{name} camera device does not exist: {camera.device}")
            if not camera.calibration.is_file():
                errors.append(f"{name} calibration does not exist: {camera.calibration}")
    if args.gpio.record_pin == args.gpio.zero_quit_pin:
        errors.append("record and zero/quit GPIO pins must be different")
    if args.gpio.hold_s <= 0 or args.gpio.bounce_s < 0 or args.gpio.toggle_debounce_s < 0:
        errors.append("GPIO hold must be positive and debounce values must be non-negative")
    if args.camera_timeout_s <= 0 or args.width_timeout_s <= 0:
        errors.append("camera and width timeouts must be positive")
    if args.preflight_timeout_s <= 0 or args.quit_burst_s < 0:
        errors.append("preflight timeout must be positive and quit burst non-negative")
    if args.max_packets is not None and args.max_packets <= 0:
        errors.append("--max-packets must be positive")
    if errors:
        raise ValueError("Pi configuration errors:\n- " + "\n- ".join(errors))


def _workers(args: Args) -> dict[str, Any]:
    if args.simulate:
        return {"left": _SimulatedWorker(80.0), "right": _SimulatedWorker(82.0)}

    from umi_teleop.camera import LocalGstCamera
    from umi_teleop.gripper import GripperConfig, GripperGeometry, detector_from_config

    result: dict[str, Any] = {}
    for name, camera_args in (("left", args.left), ("right", args.right)):
        camera = LocalGstCamera(
            camera_args.device,
            camera_args.width,
            camera_args.height,
            camera_args.fps,
            camera_args.exposure,
        )
        detector = detector_from_config(
            GripperConfig(
                geom=GripperGeometry(),
                roi_top_frac=camera_args.roi_top_frac,
                calib_path=camera_args.calibration,
            ),
            camera_args.width,
            camera_args.height,
            target_ids=camera_args.tag_ids,
        )
        result[name] = WidthWorker(camera, detector, detect_hz=camera_args.detect_hz)
    return result


def _packet(
    args: Args,
    state: PiEventState,
    source_id: str,
    seq: int,
    started_at: float,
    snapshots: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    diagnostics: list[str] = []
    arms: dict[str, dict[str, Any]] = {}
    for name in ARM_NAMES:
        snapshot = snapshots[name]
        if not snapshot["camera_ok"]:
            diagnostics.append(f"{name} camera is not producing fresh frames")
        elif not snapshot["tags_visible"]:
            diagnostics.append(f"{name} tags are not both visible")
        elif snapshot["age_s"] is None or snapshot["age_s"] > args.width_timeout_s:
            diagnostics.append(f"{name} gripper width is stale")
        if snapshot.get("last_error"):
            diagnostics.append(f"{name} camera error: {snapshot['last_error']}")
        arms[name] = {
            "width_mm": snapshot["width_mm"],
            "width_age_s": snapshot["age_s"],
            "tags_visible": snapshot["tags_visible"],
            "camera_ok": snapshot["camera_ok"],
            "frame_age_s": snapshot["frame_age_s"],
            "frames_seen": snapshot["frames_seen"],
            "detections": snapshot["detections"],
            "misses": snapshot["misses"],
            "errors": snapshot["errors"],
        }
    return {
        "version": PI_PROTOCOL_VERSION,
        "source_id": source_id,
        "session_id": state.session_id,
        "seq": seq,
        "uptime_s": max(0.0, time.monotonic() - started_at),
        "events": dict(state.counters),
        "arms": arms,
        "diagnostics": diagnostics,
    }


def run(args: Args) -> None:
    validate_args(args)
    logging.basicConfig(
        level=logging.WARNING if args.quiet else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    log = logging.getLogger("umi-pi-source")
    try:
        target_ip = socket.gethostbyname(args.network.target_host)
    except socket.gaierror as exc:
        raise RuntimeError(
            f"cannot resolve aq-yam target {args.network.target_host!r}: {exc}"
        ) from exc
    target = (target_ip, args.network.target_port)
    os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")
    workers = _workers(args)
    buttons: Any = (
        _NoButtons()
        if args.simulate
        else GpioButtons(
            record_pin=args.gpio.record_pin,
            quit_pin=args.gpio.zero_quit_pin,
            hold_s=args.gpio.hold_s,
            bounce_s=args.gpio.bounce_s,
            toggle_debounce_s=args.gpio.toggle_debounce_s,
        )
    )
    state = PiEventState()
    source_id = socket.gethostname()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.network.publish_hz
    started_at = time.monotonic()
    preflight_deadline = started_at + args.preflight_timeout_s
    ready = False
    seq = 0
    quit_at: float | None = None
    next_send = started_at
    last_status = 0.0
    try:
        for worker in workers.values():
            worker.start()
        log.info(
            "Pi source %s protocol=%d -> %s:%d",
            source_id,
            PI_PROTOCOL_VERSION,
            args.network.target_host,
            args.network.target_port,
        )
        while True:
            now = time.monotonic()
            while (event := buttons.poll()) is not None:
                state.handle(event)
                print(f"GPIO {event.name}", flush=True)
                if state.quitting and quit_at is None:
                    quit_at = now + args.quit_burst_s
            snapshots = {
                name: worker.snapshot(now, camera_timeout_s=args.camera_timeout_s)
                for name, worker in workers.items()
            }
            packet = _packet(args, state, source_id, seq, started_at, snapshots)
            sock.sendto(encode_pi_packet(packet), target)
            seq += 1
            ready = all(
                snapshots[name]["camera_ok"]
                and snapshots[name]["tags_visible"]
                and snapshots[name]["width_mm"] is not None
                and snapshots[name]["age_s"] <= args.width_timeout_s
                for name in ARM_NAMES
            )
            if ready and args.preflight_only:
                print("PI PREFLIGHT OK: both cameras and both ArUco pairs are fresh", flush=True)
                return
            if not ready and now >= preflight_deadline:
                detail = "; ".join(packet["diagnostics"]) or "sensor readiness unknown"
                raise RuntimeError(
                    f"Pi preflight timed out after {args.preflight_timeout_s:g}s: {detail}"
                )
            if not args.quiet and now - last_status >= 1.0:
                status = "ready" if ready else "waiting"
                log.info("%s seq=%d %s", status, seq, "; ".join(packet["diagnostics"]) or "fresh")
                last_status = now
            if args.max_packets is not None and seq >= args.max_packets:
                return
            if quit_at is not None and now >= quit_at:
                return
            next_send += period
            time.sleep(max(0.0, next_send - time.monotonic()))
    finally:
        for worker in reversed(tuple(workers.values())):
            worker.stop()
        buttons.close()
        sock.close()


def main() -> None:
    try:
        run(tyro.cli(Args))
    except KeyboardInterrupt:
        pass
    except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        print(f"PI SOURCE ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
