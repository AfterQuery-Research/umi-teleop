"""Typed one-command launcher for native Linux dual-YAM teleoperation."""

from __future__ import annotations

import json
import math
import os
import signal
import socket
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import tyro

from umi_teleop.dataset import DatasetConfig
from umi_teleop.vive import tracker_updates

DEFAULT_LEFT_CAN = "209D378045465008"
DEFAULT_RIGHT_CAN = "206237A845465006"


@dataclass(frozen=True)
class TrackingArgs:
    """Native libsurvive tracker input on aq-yam."""

    left_id: str = "LHR-BBAF093A"
    """Exact libsurvive serial assigned to the left UMI handle."""

    right_id: str = "LHR-C891E882"
    """Exact libsurvive serial assigned to the right UMI handle."""

    rate_hz: float = 120.0
    """Native Vive pose publication frequency."""

    fusion_hz: float = 120.0
    """Fused Vive/Pi telemetry frequency sent to the YAM controller."""

    bind: str = "127.0.0.1"
    """Loopback address used by the native Vive receiver."""

    port: int = 9001
    """Local UDP port carrying native Vive poses into sensor fusion."""

    stale_timeout_s: float = 0.20
    """Maximum Vive pose age allowed while arming or controlling."""

    max_linear_speed_m_s: float = 8.0
    """Reject native tracker samples faster than this linear speed."""

    max_angular_speed_deg_s: float = 1440.0
    """Reject native tracker samples faster than this angular speed."""

    reacquire_ms: float = 250.0
    """Continuous plausible tracking required after a rejected sample."""

    poser: str = "KalmanOnly"
    """libsurvive pose solver: stable KalmanOnly default or expert MPFIT override."""

    startup_timeout_s: float = 8.0
    """Per-attempt timeout for both trackers to become continuously valid."""

    startup_stable_s: float = 0.75
    """Continuous two-tracker validity required before fusion and control start."""

    startup_attempts: int = 3
    """Tracker-only startup attempts allowed before any controller process starts."""

    simulate: bool = False
    """Use a deterministic synthetic Vive sender for non-moving tests."""


@dataclass(frozen=True)
class PiInputArgs:
    """Raspberry Pi GPIO and ArUco telemetry received by aq-yam."""

    source_host: str = "right-umi-vive-teleop.local"
    """Expected Pi hostname or IPv4 address; packets from other hosts are ignored."""

    bind: str = "0.0.0.0"
    """aq-yam address on which to receive Pi UDP telemetry."""

    port: int = 9139
    """aq-yam UDP port on which to receive Pi telemetry."""

    heartbeat_timeout_s: float = 0.50
    """Maximum age of the newest Pi packet before both arms disarm."""

    width_timeout_s: float = 0.75
    """Maximum end-to-end ArUco width age before both arms disarm."""


@dataclass(frozen=True)
class RobotArgs:
    """Dual YAM hardware identity and gripper calibration."""

    left_can_serial: str = os.environ.get("LEFT_CAN", DEFAULT_LEFT_CAN)
    """USB serial of the CAN adapter wired to the left YAM."""

    right_can_serial: str = os.environ.get("RIGHT_CAN", DEFAULT_RIGHT_CAN)
    """USB serial of the CAN adapter wired to the right YAM."""

    left_gripper_limits: tuple[float, float] = (6.3704509, 1.2239643)
    """Left gripper raw closed/open motor endpoints."""

    right_gripper_limits: tuple[float, float] = (6.3967727, 1.2010758)
    """Right gripper raw closed/open motor endpoints."""

    left_input_width_mm: tuple[float, float] = (40.376321, 122.999636)
    """Measured left UMI gripper widths mapped to closed/open robot commands."""

    right_input_width_mm: tuple[float, float] = (40.128507, 122.685643)
    """Measured right UMI gripper widths mapped to closed/open robot commands."""


@dataclass(frozen=True)
class ControlArgs:
    """Responsiveness, filtering, workspace, and IK behavior."""

    controller_hz: float = 100.0
    """Dual-arm state-read, IK, safety, and command frequency."""

    telemetry_bind: str = "127.0.0.1"
    """Loopback address used by fusion and the controller."""

    telemetry_port: int = 9140
    """Loopback UDP port carrying fused telemetry to the controller."""

    position_scale: float = 2.0
    """Multiplier applied to tracker translation relative to ZERO."""

    rotation_scale: float = 1.0
    """Multiplier applied to tracker rotation relative to ZERO."""

    max_joint_speed: float = 1.0
    """Per-joint velocity limit in radians per second."""

    max_joint_acceleration: float = 8.0
    """Per-joint acceleration limit in radians per second squared."""

    translation_filter_hz: float = 18.0
    """Translation low-pass cutoff in Hz; zero disables filtering."""

    rotation_filter_hz: float = 15.0
    """Rotation low-pass cutoff in Hz; zero disables filtering."""

    translation_deadband_m: float = 0.001
    """Tracker translation changes below this distance are ignored."""

    rotation_deadband_deg: float = 0.25
    """Tracker rotation changes below this angle are ignored."""

    max_translation_m: float = 0.50
    """Maximum Cartesian displacement from each arm's ZERO pose."""

    max_rotation_deg: float = 90.0
    """Maximum orientation displacement from each arm's ZERO pose."""

    max_zero_translation_m: float = 0.75
    """Raw tracker displacement from ZERO that invalidates the captured ZERO."""

    max_zero_rotation_deg: float = 150.0
    """Raw tracker rotation from ZERO that invalidates the captured ZERO."""

    synchronous_ik: bool = True
    """Solve IK in the current control cycle; disable for background one-cycle-late IK."""

    ik_max_iters: int = 40
    """Maximum numerical IK iterations per solve attempt."""

    ik_fallback_attempts_per_cycle: int = 2
    """Maximum fallback target attempts after a direct IK solve fails."""


@dataclass(frozen=True)
class Args:
    """Run Pi-assisted native Vive teleoperation and optional LeRobot recording."""

    tracking: TrackingArgs = field(default_factory=TrackingArgs)
    pi: PiInputArgs = field(default_factory=PiInputArgs)
    robot: RobotArgs = field(default_factory=RobotArgs)
    control: ControlArgs = field(default_factory=ControlArgs)
    dataset: DatasetConfig = field(default_factory=DatasetConfig)
    confirm_clear: bool = False
    """Confirm that the complete physical arm workspace is clear."""

    preflight_only: bool = False
    """Validate sensors, CAN, cameras, dataset, and authentication without opening robots."""

    simulate: bool = False
    """Use I2RT simulation and never open physical CAN devices."""

    quiet: bool = True
    """Show only state changes, warnings, and errors; pass --no-quiet for diagnostics."""


Config = Args


def validate_args(args: Args) -> None:
    errors: list[str] = []
    if sys.platform != "linux":
        errors.append("native local teleoperation requires Linux")
    if not args.simulate and not args.preflight_only and not args.confirm_clear:
        errors.append("clear the complete arm workspace and pass --confirm-clear")
    if not args.tracking.left_id.strip() or not args.tracking.right_id.strip():
        errors.append("left and right Vive tracker IDs must not be empty")
    if args.tracking.left_id.casefold() == args.tracking.right_id.casefold():
        errors.append("left and right Vive tracker IDs must be different")
    if not args.pi.source_host.strip():
        errors.append("Pi source host must not be empty")
    for name, port in (
        ("tracking", args.tracking.port),
        ("Pi", args.pi.port),
        ("controller telemetry", args.control.telemetry_port),
    ):
        if not 1 <= port <= 65_535:
            errors.append(f"{name} UDP port must be between 1 and 65535")
    for name, value, low, high in (
        ("tracking rate", args.tracking.rate_hz, 30.0, 240.0),
        ("fusion rate", args.tracking.fusion_hz, 30.0, 240.0),
        ("controller rate", args.control.controller_hz, 30.0, 120.0),
        ("position scale", args.control.position_scale, 0.01, 3.0),
        ("rotation scale", args.control.rotation_scale, 0.01, 2.0),
        ("maximum joint speed", args.control.max_joint_speed, 0.01, 2.0),
        ("maximum joint acceleration", args.control.max_joint_acceleration, 0.01, 30.0),
        ("translation filter", args.control.translation_filter_hz, 0.0, 60.0),
        ("rotation filter", args.control.rotation_filter_hz, 0.0, 60.0),
        ("tracking linear speed", args.tracking.max_linear_speed_m_s, 0.1, 100.0),
        ("tracking angular speed", args.tracking.max_angular_speed_deg_s, 10.0, 10_000.0),
        ("tracking reacquisition", args.tracking.reacquire_ms, 10.0, 5_000.0),
        ("tracking startup timeout", args.tracking.startup_timeout_s, 2.0, 60.0),
        ("tracking startup stability", args.tracking.startup_stable_s, 0.1, 5.0),
    ):
        if not math.isfinite(value) or not low <= value <= high:
            errors.append(f"{name} must be between {low:g} and {high:g}")
    if args.control.translation_deadband_m < 0:
        errors.append("translation deadband must be non-negative")
    if not 1 <= args.tracking.startup_attempts <= 5:
        errors.append("tracking startup attempts must be between 1 and 5")
    if args.tracking.poser not in {"KalmanOnly", "MPFIT"}:
        errors.append("tracking poser must be KalmanOnly or MPFIT")
    if not 0 <= args.control.rotation_deadband_deg <= 10:
        errors.append("rotation deadband must be between 0 and 10 degrees")
    if not 0 < args.control.max_translation_m <= 1.0:
        errors.append("maximum translation must be greater than zero and at most 1 meter")
    if not 0 < args.control.max_rotation_deg <= 180:
        errors.append("maximum rotation must be greater than zero and at most 180 degrees")
    if not 0 < args.control.max_zero_translation_m <= 2.0:
        errors.append(
            "maximum ZERO residual translation must be greater than zero and at most 2 meters"
        )
    if not 0 < args.control.max_zero_rotation_deg <= 180:
        errors.append(
            "maximum ZERO residual rotation must be greater than zero and at most 180 degrees"
        )
    if args.pi.heartbeat_timeout_s <= 0 or args.pi.width_timeout_s <= 0:
        errors.append("Pi heartbeat and width timeouts must be positive")
    if not args.simulate:
        if not args.robot.left_can_serial.strip() or not args.robot.right_can_serial.strip():
            errors.append("left and right CAN adapter serials must not be empty")
        if args.robot.left_can_serial.casefold() == args.robot.right_can_serial.casefold():
            errors.append("left and right CAN adapter serials must be different")
    if args.dataset.enabled:
        if not args.dataset.repo_id:
            errors.append("dataset recording requires --dataset.repo-id OWNER/NAME")
        if not args.dataset.task or not args.dataset.task.strip():
            errors.append("dataset recording requires --dataset.task")
    if errors:
        raise ValueError("teleoperation configuration errors:\n- " + "\n- ".join(errors))


def _append_dataset(command: list[str], dataset: DatasetConfig) -> None:
    if not dataset.enabled:
        return
    values: tuple[tuple[str, Any], ...] = (
        ("repo-id", dataset.repo_id),
        ("root", dataset.root),
        ("task", dataset.task),
        ("top-camera-serial", dataset.top_camera_serial),
        ("left-camera-serial", dataset.left_camera_serial),
        ("right-camera-serial", dataset.right_camera_serial),
        ("side-camera-kind", dataset.side_camera_kind),
        ("left-camera-path", dataset.left_camera_path),
        ("right-camera-path", dataset.right_camera_path),
        ("recording-fps", dataset.recording_fps),
        ("post-episode-action", dataset.post_episode_action),
        ("min-episode-frames", dataset.min_episode_frames),
        ("queue-size", dataset.queue_size),
        ("camera-max-age-ms", dataset.camera_max_age_ms),
        ("min-free-disk-gb", dataset.min_free_disk_gb),
        ("encoder-threads", dataset.encoder_threads),
        ("encoder-queue-size", dataset.encoder_queue_size),
        ("video-codec", dataset.video_codec),
    )
    command.append("--dataset.record")
    for name, value in values:
        if value is not None:
            command.extend((f"--dataset.{name}", str(value)))
    for name, enabled in (
        ("resume", dataset.resume),
        ("push-to-hub", dataset.push_to_hub),
        ("private", dataset.private),
        ("streaming-encoding", dataset.streaming_encoding),
    ):
        command.append(f"--dataset.{name}" if enabled else f"--dataset.no-{name}")


def build_commands(args: Args, bin_dir: Path | None = None) -> dict[str, list[str]]:
    validate_args(args)
    binaries = Path(sys.executable).parent if bin_dir is None else bin_dir
    tracking_executable = "umi-vive-sim" if args.tracking.simulate else "umi-vive-linux"
    commands = {
        "tracking": [
            str(binaries / tracking_executable),
            "--target-host",
            args.tracking.bind,
            "--target-port",
            str(args.tracking.port),
            "--rate-hz",
            f"{args.tracking.rate_hz:g}",
        ],
        "source": [
            str(binaries / "umi-source"),
            "--target-host",
            args.control.telemetry_bind,
            "--target-port",
            str(args.control.telemetry_port),
            "--vive-bind",
            args.tracking.bind,
            "--vive-port",
            str(args.tracking.port),
            "--left-serial",
            args.tracking.left_id,
            "--right-serial",
            args.tracking.right_id,
            "--pi-bind",
            args.pi.bind,
            "--pi-port",
            str(args.pi.port),
            "--pi-expected-host",
            args.pi.source_host,
            "--pi-heartbeat-timeout-s",
            f"{args.pi.heartbeat_timeout_s:g}",
            "--max-width-age-s",
            f"{args.pi.width_timeout_s:g}",
            "--max-pose-age-s",
            f"{args.tracking.stale_timeout_s:g}",
            "--send-hz",
            f"{args.tracking.fusion_hz:g}",
            "--buttons",
            "pi",
            "--enable-confirm-s",
            "1.0",
            "--enable-cooldown-s",
            "0.5",
        ],
        "controller": [
            str(binaries / "umi-yam"),
            "--bind",
            args.control.telemetry_bind,
            "--port",
            str(args.control.telemetry_port),
            "--hz",
            f"{args.control.controller_hz:g}",
            "--position-scale",
            f"{args.control.position_scale:g}",
            "--rotation-scale",
            f"{args.control.rotation_scale:g}",
            "--max-joint-speed",
            f"{args.control.max_joint_speed:g}",
            "--max-joint-acceleration",
            f"{args.control.max_joint_acceleration:g}",
            "--translation-filter-hz",
            f"{args.control.translation_filter_hz:g}",
            "--rotation-filter-hz",
            f"{args.control.rotation_filter_hz:g}",
            "--translation-deadband-m",
            f"{args.control.translation_deadband_m:g}",
            "--rotation-deadband-deg",
            f"{args.control.rotation_deadband_deg:g}",
            "--max-translation",
            f"{args.control.max_translation_m:g}",
            "--max-rotation-deg",
            f"{args.control.max_rotation_deg:g}",
            "--max-zero-translation",
            f"{args.control.max_zero_translation_m:g}",
            "--max-zero-rotation-deg",
            f"{args.control.max_zero_rotation_deg:g}",
            "--max-packet-age",
            "0.25",
            "--max-pose-age",
            f"{args.tracking.stale_timeout_s:g}",
            "--max-width-age",
            f"{args.pi.width_timeout_s:g}",
            "--left-gripper-limits",
            *(str(value) for value in args.robot.left_gripper_limits),
            "--right-gripper-limits",
            *(str(value) for value in args.robot.right_gripper_limits),
            "--left-input-width-mm",
            *(str(value) for value in args.robot.left_input_width_mm),
            "--right-input-width-mm",
            *(str(value) for value in args.robot.right_input_width_mm),
            "--ik-max-iters",
            str(args.control.ik_max_iters),
            "--ik-fallback-attempts-per-cycle",
            str(args.control.ik_fallback_attempts_per_cycle),
        ],
    }
    if args.quiet:
        commands["source"].append("--quiet")
        commands["controller"].append("--quiet")
    if args.tracking.simulate:
        commands["tracking"].extend(
            ["--left-id", args.tracking.left_id, "--right-id", args.tracking.right_id]
        )
    else:
        commands["tracking"].extend(
            [
                "--max-linear-speed-m-s",
                f"{args.tracking.max_linear_speed_m_s:g}",
                "--max-angular-speed-deg-s",
                f"{args.tracking.max_angular_speed_deg_s:g}",
                "--reacquire-ms",
                f"{args.tracking.reacquire_ms:g}",
                "--poser",
                args.tracking.poser,
            ]
        )
    if args.simulate:
        commands["controller"].append("--simulate")
    else:
        commands["controller"].extend(
            [
                "--left-adapter-serial",
                args.robot.left_can_serial,
                "--right-adapter-serial",
                args.robot.right_can_serial,
            ]
        )
        if args.confirm_clear:
            commands["controller"].append("--confirm-clear")
    if args.control.synchronous_ik:
        commands["controller"].append("--synchronous-ik")
    if args.preflight_only:
        commands["controller"].append("--preflight-only")
    _append_dataset(commands["controller"], args.dataset)
    return commands


def stop_processes(processes: list[tuple[str, Any]], timeout_s: float = 5.0) -> None:
    for _name, process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and any(
        process.poll() is None for _name, process in processes
    ):
        time.sleep(0.05)
    for _name, process in reversed(processes):
        if process.poll() is None:
            process.terminate()
    for _name, process in reversed(processes):
        if process.poll() is None:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)


def coordinated_shutdown_status(
    processes: list[tuple[str, Any]], timeout_s: float = 2.0
) -> int | None:
    """Recognize GPIO QUIT after both quit-aware children exit cleanly."""
    deadline = time.monotonic() + timeout_s
    while True:
        statuses = {name: process.poll() for name, process in processes}
        failures = [status for status in statuses.values() if status not in (None, 0)]
        if failures:
            return int(failures[0])
        if statuses.get("source") == 0 and statuses.get("controller") == 0:
            return 0
        if time.monotonic() >= deadline:
            return None
        time.sleep(0.05)


def wait_for_tracking_ready(
    receiver: socket.socket,
    process: Any,
    *,
    expected_serials: set[str],
    timeout_s: float,
    stable_s: float,
) -> None:
    """Wait for both tracker serials to remain valid before any controller child starts."""
    deadline = time.monotonic() + timeout_s
    stable_since: float | None = None
    stable_continuity: tuple[tuple[str, int | None], ...] | None = None
    last_sequence: int | None = None
    latest_valid_serials: set[str] = set()
    continuity_changes = 0
    last_problem = "no native tracking packets received"
    receiver.settimeout(min(0.25, timeout_s))
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(f"native tracking exited with status {return_code} during startup")
        try:
            payload, _address = receiver.recvfrom(65_535)
            packet = json.loads(payload)
            if not isinstance(packet, dict) or packet.get("frame") != "libsurvive_tracking":
                raise ValueError("not a native libsurvive tracking packet")
            sequence = packet.get("seq")
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                raise ValueError("native tracking sequence must be a non-negative integer")
            if last_sequence is not None and sequence <= last_sequence:
                raise ValueError("native tracking sequence did not advance")
            updates = tracker_updates(packet)
        except TimeoutError:
            stable_since = None
            stable_continuity = None
            last_problem = "native tracking packet stream timed out"
            continue
        except (
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
            UnicodeDecodeError,
        ) as exc:
            stable_since = None
            stable_continuity = None
            last_problem = f"invalid native tracking packet: {exc}"
            continue

        last_sequence = sequence
        valid = {
            serial: update
            for serial, update in updates.items()
            if update.pose is not None and update.continuity_id is not None
        }
        latest_valid_serials = expected_serials.intersection(valid)
        if not expected_serials.issubset(valid):
            stable_since = None
            stable_continuity = None
            missing = sorted(expected_serials - latest_valid_serials)
            last_problem = f"missing or invalid: {', '.join(missing)}"
            continue
        continuity = tuple(
            sorted((serial, valid[serial].continuity_id) for serial in expected_serials)
        )
        last_problem = "both trackers did not stay continuously valid"
        now = time.monotonic()
        if stable_since is None or continuity != stable_continuity:
            if stable_since is not None:
                continuity_changes += 1
            stable_since = now
            stable_continuity = continuity
        elif now - stable_since >= stable_s:
            return
    missing = sorted(expected_serials - latest_valid_serials)
    if missing:
        detail = f"missing or invalid: {', '.join(missing)}"
    elif continuity_changes:
        detail = f"continuity changed {continuity_changes} time(s)"
    else:
        detail = last_problem
    raise RuntimeError(
        f"trackers did not remain valid for {stable_s:g}s within {timeout_s:g}s; {detail}"
    )


def start_tracking_process(
    args: Args,
    command: list[str],
    *,
    popen: Callable[..., Any] = subprocess.Popen,
) -> Any:
    """Start and, before control exists, retry native tracking until both poses are stable."""
    expected_serials = {args.tracking.left_id, args.tracking.right_id}
    last_error: RuntimeError | None = None
    for attempt in range(1, args.tracking.startup_attempts + 1):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            receiver.bind((args.tracking.bind, args.tracking.port))
        except OSError as exc:
            receiver.close()
            raise RuntimeError(
                f"cannot bind tracking startup probe on "
                f"{args.tracking.bind}:{args.tracking.port}: {exc}"
            ) from exc
        process: Any | None = None
        try:
            process = popen(command)
            wait_for_tracking_ready(
                receiver,
                process,
                expected_serials=expected_serials,
                timeout_s=args.tracking.startup_timeout_s,
                stable_s=args.tracking.startup_stable_s,
            )
            print(
                f"tracking ready: both configured trackers stable for "
                f"{args.tracking.startup_stable_s:g}s",
                flush=True,
            )
            return process
        except RuntimeError as exc:
            last_error = exc
            if process is not None:
                stop_processes([("tracking", process)], timeout_s=2.0)
            if attempt < args.tracking.startup_attempts:
                print(
                    f"tracking startup attempt {attempt} failed: {exc}; retrying before control",
                    file=sys.stderr,
                    flush=True,
                )
                time.sleep(0.5)
        except BaseException:
            if process is not None:
                stop_processes([("tracking", process)], timeout_s=2.0)
            raise
        finally:
            receiver.close()
    raise RuntimeError(
        f"tracking startup failed after {args.tracking.startup_attempts} attempts: {last_error}"
    )


def run(args: Args, *, popen: Callable[..., Any] = subprocess.Popen) -> int:
    commands = build_commands(args)
    for name, command in commands.items():
        executable = Path(command[0])
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError(f"missing {name} executable: {executable}")

    print("UMI TELEOP: local Vive + Pi GPIO/ArUco + dual YAM", flush=True)
    print(
        "GPIO: ZERO tap=zero, RECORD tap=start/stop, RECORD hold=cancel, ZERO hold=quit",
        flush=True,
    )
    print(
        f"Profile: translation={args.control.position_scale:g}x "
        f"rotation={args.control.rotation_scale:g}x "
        f"joint_speed={args.control.max_joint_speed:g}rad/s "
        f"control={args.control.controller_hz:g}Hz dataset="
        f"{'on' if args.dataset.enabled else 'off'}",
        flush=True,
    )
    processes: list[tuple[str, Any]] = []
    try:
        print("starting tracking...", flush=True)
        tracking_process = start_tracking_process(args, commands["tracking"], popen=popen)
        processes.append(("tracking", tracking_process))
        for name in ("source", "controller"):
            print(f"starting {name}...", flush=True)
            processes.append((name, popen(commands[name])))
        while True:
            for name, process in processes:
                return_code = process.poll()
                if return_code is not None:
                    if return_code != 0:
                        print(f"{name} exited with status {return_code}", file=sys.stderr)
                    elif not args.preflight_only:
                        coordinated_status = coordinated_shutdown_status(processes)
                        if coordinated_status is None:
                            print(f"{name} exited unexpectedly", file=sys.stderr)
                            return_code = 1
                        else:
                            return_code = coordinated_status
                    return int(return_code)
            time.sleep(0.1)
    except KeyboardInterrupt:
        return 130
    finally:
        stop_processes(processes)


def main() -> None:
    try:
        raise SystemExit(run(tyro.cli(Args)))
    except (RuntimeError, ValueError) as exc:
        print(f"TELEOP START ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
