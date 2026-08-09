"""Read-only Vive tracker monitoring through libsurvive's Simple API."""

from __future__ import annotations

import ctypes
import ctypes.util
import math
import os
import threading
import time
from collections.abc import Callable
from contextlib import closing
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import tyro

from umi_teleop_fixed.calibration import cache_root
from umi_teleop_fixed.lighthouses import (
    load_lighthouse_status,
    require_lighthouse_calibration,
)

_POSE_EVENT = 3
_SHUTDOWN_EVENT = 4
_TRACKED_OBJECT = 3


class _SurvivePose(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_double * 3),
        ("rotation", ctypes.c_double * 4),
    ]


class _SurviveVelocity(ctypes.Structure):
    _fields_ = [
        ("linear", ctypes.c_double * 3),
        ("angular", ctypes.c_double * 3),
    ]


class _PoseEvent(ctypes.Structure):
    _fields_ = [
        ("timestamp", ctypes.c_double),
        ("object", ctypes.c_void_p),
        ("pose", _SurvivePose),
        ("velocity", _SurviveVelocity),
    ]


class _EventPayload(ctypes.Union):
    _fields_ = [
        ("alignment", ctypes.c_void_p),
        ("storage", ctypes.c_byte * 128),
    ]


class _SurviveEvent(ctypes.Structure):
    _fields_ = [
        ("event_type", ctypes.c_int),
        ("payload", _EventPayload),
    ]


@dataclass(frozen=True)
class TrackerPose:
    """One timestamped pose in libsurvive's tracking frame."""

    name: str
    serial: str
    timestamp: float
    position: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        name = self.name.strip()
        serial = self.serial.strip()
        values = (self.timestamp, *self.position, *self.quaternion_wxyz)
        if not name or not serial:
            raise ValueError("tracker name and serial must not be empty")
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"tracker {serial} produced a non-finite pose")
        if self.timestamp <= 0:
            raise ValueError(f"tracker {serial} produced an invalid timestamp")

        norm = math.sqrt(sum(value * value for value in self.quaternion_wxyz))
        if norm < 1e-6:
            raise ValueError(f"tracker {serial} produced an invalid quaternion")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "serial", serial)
        object.__setattr__(
            self,
            "quaternion_wxyz",
            tuple(value / norm for value in self.quaternion_wxyz),
        )


@dataclass(frozen=True)
class TrackerReport:
    """Current health and latest pose for one tracker."""

    role: str | None
    pose: TrackerPose
    updates: int
    rate_hz: float
    age_seconds: float


@dataclass(frozen=True)
class TrackerDelta:
    """Pose displacement from a per-tracker reference sample."""

    translation_mm: tuple[float, float, float]
    rotation_degrees: float


@dataclass
class _TrackerState:
    role: str | None
    pose: TrackerPose
    updates: int
    first_received: float
    last_received: float


class TrackerMonitor:
    """Assign tracker roles and calculate freshness and update rates."""

    def __init__(self, *, left_serial: str | None, right_serial: str | None) -> None:
        requested = {}
        if left_serial is not None and right_serial is not None:
            requested = {left_serial.casefold(): "left", right_serial.casefold(): "right"}
        self._requested = requested
        self._states: dict[str, _TrackerState] = {}

    def observe(self, pose: TrackerPose, *, received_at: float) -> bool:
        """Record a new timestamp for a selected tracker."""
        key = pose.serial.casefold()
        if self._requested and key not in self._requested:
            return False
        state = self._states.get(key)
        if state is not None and pose.timestamp <= state.pose.timestamp:
            return False
        if state is None:
            self._states[key] = _TrackerState(
                role=self._requested.get(key),
                pose=pose,
                updates=1,
                first_received=received_at,
                last_received=received_at,
            )
        else:
            state.pose = pose
            state.updates += 1
            state.last_received = received_at
        return True

    def reports(self, *, now: float) -> tuple[TrackerReport, ...]:
        reports = []
        for state in self._states.values():
            elapsed = state.last_received - state.first_received
            rate_hz = (state.updates - 1) / elapsed if elapsed > 0 else 0.0
            reports.append(
                TrackerReport(
                    role=state.role,
                    pose=state.pose,
                    updates=state.updates,
                    rate_hz=rate_hz,
                    age_seconds=max(0.0, now - state.last_received),
                )
            )
        return tuple(sorted(reports, key=lambda report: (report.role or "", report.pose.serial)))

    def health_errors(
        self,
        *,
        now: float,
        expected_count: int,
        stale_after_seconds: float,
        min_rate_hz: float,
    ) -> tuple[str, ...]:
        reports = self.reports(now=now)
        errors = []
        if self._requested:
            observed_roles = {report.role for report in reports}
            for role in ("left", "right"):
                if role not in observed_roles:
                    errors.append(f"no poses received for {role} tracker")
        elif len(reports) != expected_count:
            errors.append(
                f"expected {expected_count} tracked devices, received poses from {len(reports)}"
            )
        for report in reports:
            label = report.role or report.pose.serial
            if report.age_seconds > stale_after_seconds:
                errors.append(f"{label} tracker pose is stale ({report.age_seconds:.3f}s old)")
            if report.rate_hz < min_rate_hz:
                errors.append(f"{label} tracker update rate is too low ({report.rate_hz:.1f} Hz)")
        return tuple(errors)


class PoseSource(Protocol):
    def next_pose(self) -> TrackerPose | None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class TrackerSample:
    """Latest pose plus the local monotonic time at which it arrived."""

    pose: TrackerPose
    received_at: float


class TrackerReader:
    """Continuously drain libsurvive so the control loop never blocks on tracking."""

    def __init__(
        self,
        source: PoseSource,
        *,
        serials: dict[str, str],
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        normalized = {side: serial.strip() for side, serial in serials.items()}
        if set(normalized) != {"left", "right"} or any(not value for value in normalized.values()):
            raise ValueError("tracker reader requires non-empty left and right serials")
        if normalized["left"].casefold() == normalized["right"].casefold():
            raise ValueError("left and right tracker serials must be different")
        self._source = source
        self._roles = {serial.casefold(): side for side, serial in normalized.items()}
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._samples: dict[str, TrackerSample] = {}
        self._error: BaseException | None = None
        self._stop = threading.Event()
        self._closed = False
        self._thread = threading.Thread(
            target=self._read_loop,
            name="umi-libsurvive-reader",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def _read_loop(self) -> None:
        try:
            while not self._stop.is_set():
                pose = self._source.next_pose()
                if pose is None:
                    continue
                side = self._roles.get(pose.serial.casefold())
                if side is None:
                    continue
                sample = TrackerSample(pose=pose, received_at=self._monotonic())
                with self._lock:
                    previous = self._samples.get(side)
                    if previous is None or pose.timestamp > previous.pose.timestamp:
                        self._samples[side] = sample
        except BaseException as error:
            if not self._stop.is_set():
                with self._lock:
                    self._error = error

    def snapshot(self, *, now: float, stale_after_seconds: float) -> dict[str, TrackerSample]:
        with self._lock:
            if self._error is not None:
                raise RuntimeError(f"libsurvive reader failed: {self._error}") from self._error
            samples = dict(self._samples)
        missing = [side for side in ("left", "right") if side not in samples]
        if missing:
            raise RuntimeError(f"no live pose for {', '.join(missing)} tracker")
        for side, sample in samples.items():
            age = max(0.0, now - sample.received_at)
            if age > stale_after_seconds:
                raise RuntimeError(f"{side} tracker pose is stale ({age:.3f}s old)")
        return samples

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self._thread.join(timeout=0.5)
        self._source.close()
        self._thread.join(timeout=1.0)
        if self._thread.is_alive():
            raise RuntimeError("libsurvive reader did not stop cleanly")

    def __enter__(self) -> TrackerReader:
        self.start()
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.close()


def _load_libsurvive(path: Path | None) -> ctypes.CDLL:
    candidates: list[str] = []
    if path is not None:
        candidates.append(str(path.expanduser()))
    elif configured := os.environ.get("LIBSURVIVE_PATH"):
        candidates.append(str(Path(configured).expanduser()))
    else:
        if discovered := ctypes.util.find_library("survive"):
            candidates.append(discovered)
        candidates.extend(
            (
                str(Path.home() / ".local" / "lib" / "libsurvive.so.0"),
                "/usr/local/lib/libsurvive.so.0",
                "libsurvive.so.0",
                "libsurvive.so",
            )
        )

    errors = []
    for candidate in candidates:
        try:
            return ctypes.CDLL(candidate, mode=ctypes.RTLD_GLOBAL)
        except OSError as error:
            errors.append(f"{candidate}: {error}")
    detail = "; ".join(errors) if errors else "no library candidates found"
    raise RuntimeError(
        "Could not load libsurvive. Install it system-wide or pass --libsurvive-path. "
        f"Tried: {detail}"
    )


def _libsurvive_arguments(
    *,
    config_path: Path,
    poser: str,
    disable_calibration: bool,
    force_calibration: bool,
    lighthouse_count: int | None = None,
) -> list[bytes]:
    if disable_calibration and force_calibration:
        raise ValueError("calibration cannot be both disabled and forced")
    if lighthouse_count is not None and lighthouse_count <= 0:
        raise ValueError("lighthouse_count must be positive")
    arguments = [
        b"umi-teleop-fixed",
        b"-p",
        poser.encode(),
        b"-c",
        os.fsencode(config_path),
        b"--v",
        b"0",
    ]
    if disable_calibration:
        arguments.extend((b"--disable-calibrate", b"1"))
    if force_calibration:
        arguments.extend((b"--force-calibrate", b"1"))
    if lighthouse_count is not None:
        arguments.extend((b"-l", str(lighthouse_count).encode()))
    return arguments


class LibsurviveSource:
    """Minimal ctypes binding to libsurvive's thread-safe event interface."""

    def __init__(
        self,
        *,
        library_path: Path | None,
        config_path: Path,
        poser: str,
        disable_calibration: bool = False,
        force_calibration: bool = False,
        lighthouse_count: int | None = None,
    ) -> None:
        self._library = _load_libsurvive(library_path)
        self._configure_functions()
        config_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        arguments = _libsurvive_arguments(
            config_path=config_path,
            poser=poser,
            disable_calibration=disable_calibration,
            force_calibration=force_calibration,
            lighthouse_count=lighthouse_count,
        )
        self._argv = (ctypes.c_char_p * len(arguments))(*arguments)
        self._context = self._library.survive_simple_init(len(arguments), self._argv)
        if not self._context:
            raise RuntimeError("libsurvive initialization failed")
        self._library.survive_simple_start_thread(self._context)

    def _configure_functions(self) -> None:
        library = self._library
        library.survive_simple_init.argtypes = [
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_char_p),
        ]
        library.survive_simple_init.restype = ctypes.c_void_p
        library.survive_simple_start_thread.argtypes = [ctypes.c_void_p]
        library.survive_simple_start_thread.restype = None
        library.survive_simple_wait_for_event.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_SurviveEvent),
        ]
        library.survive_simple_wait_for_event.restype = ctypes.c_int
        library.survive_simple_get_pose_updated_event.argtypes = [ctypes.POINTER(_SurviveEvent)]
        library.survive_simple_get_pose_updated_event.restype = ctypes.POINTER(_PoseEvent)
        library.survive_simple_object_get_type.argtypes = [ctypes.c_void_p]
        library.survive_simple_object_get_type.restype = ctypes.c_int
        library.survive_simple_object_name.argtypes = [ctypes.c_void_p]
        library.survive_simple_object_name.restype = ctypes.c_char_p
        library.survive_simple_serial_number.argtypes = [ctypes.c_void_p]
        library.survive_simple_serial_number.restype = ctypes.c_char_p
        library.survive_simple_close.argtypes = [ctypes.c_void_p]
        library.survive_simple_close.restype = None

    def next_pose(self) -> TrackerPose | None:
        if self._context is None:
            raise RuntimeError("libsurvive source is closed")
        event = _SurviveEvent()
        event_type = self._library.survive_simple_wait_for_event(self._context, ctypes.byref(event))
        if event_type == _SHUTDOWN_EVENT:
            raise RuntimeError("libsurvive stopped while tracking")
        if event_type != _POSE_EVENT:
            return None
        updated = self._library.survive_simple_get_pose_updated_event(ctypes.byref(event))
        if not updated:
            return None
        pose_event = updated.contents
        if self._library.survive_simple_object_get_type(pose_event.object) != _TRACKED_OBJECT:
            return None
        name = self._library.survive_simple_object_name(pose_event.object)
        serial = self._library.survive_simple_serial_number(pose_event.object)
        return TrackerPose(
            name=(name or b"").decode(errors="replace"),
            serial=(serial or b"").decode(errors="replace"),
            timestamp=float(pose_event.timestamp),
            position=tuple(float(value) for value in pose_event.pose.position),
            quaternion_wxyz=tuple(float(value) for value in pose_event.pose.rotation),
        )

    def close(self) -> None:
        if self._context is not None:
            context = self._context
            self._context = None
            self._library.survive_simple_close(context)


@dataclass(frozen=True)
class Args:
    """Discover and validate two Vive trackers without commanding a robot."""

    left_tracker_serial: str | None = None
    """Stable serial for the left tracker; must be supplied with the right serial."""

    right_tracker_serial: str | None = None
    """Stable serial for the right tracker; must be supplied with the left serial."""

    duration_seconds: float = 5.0
    """How long to observe poses before reporting health."""

    stale_after_seconds: float = 0.25
    """Maximum acceptable age of the latest pose."""

    min_rate_hz: float = 20.0
    """Minimum acceptable pose update rate per tracker."""

    print_rate_hz: float = 2.0
    """Maximum rate for human-readable pose output."""

    libsurvive_path: Path | None = None
    """Path to libsurvive.so when it is not installed in the system library path."""

    config_path: Path | None = None
    """Libsurvive configuration path; defaults to the user cache."""

    poser: str = "MPFIT"
    """Libsurvive poser plugin."""

    def validated_serials(self) -> tuple[str | None, str | None]:
        left = self.left_tracker_serial.strip() if self.left_tracker_serial else None
        right = self.right_tracker_serial.strip() if self.right_tracker_serial else None
        if (left is None) != (right is None):
            raise ValueError("left and right tracker serials must be supplied together")
        if left is not None and left.casefold() == right.casefold():
            raise ValueError("left and right tracker serials must be different")
        for name, value in (
            ("duration_seconds", self.duration_seconds),
            ("stale_after_seconds", self.stale_after_seconds),
            ("min_rate_hz", self.min_rate_hz),
            ("print_rate_hz", self.print_rate_hz),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if not self.poser.strip():
            raise ValueError("poser must not be empty")
        return left, right

    def resolved_config_path(self) -> Path:
        if self.config_path is not None:
            return self.config_path.expanduser()
        return cache_root() / "umi-teleop-fixed" / "libsurvive" / "config.json"


SourceFactory = Callable[[Args], PoseSource]


def _create_source(args: Args) -> PoseSource:
    config_path = args.resolved_config_path()
    require_lighthouse_calibration(config_path)
    return LibsurviveSource(
        library_path=args.libsurvive_path,
        config_path=config_path,
        poser=args.poser,
        disable_calibration=True,
    )


def tracker_delta(pose: TrackerPose, reference: TrackerPose) -> TrackerDelta:
    """Return translation and shortest rotation displacement from a reference pose."""
    if pose.serial.casefold() != reference.serial.casefold():
        raise ValueError("pose and reference must belong to the same tracker")
    translation_mm = tuple(
        (current - initial) * 1000.0
        for current, initial in zip(pose.position, reference.position, strict=True)
    )
    dot = sum(
        current * initial
        for current, initial in zip(
            pose.quaternion_wxyz,
            reference.quaternion_wxyz,
            strict=True,
        )
    )
    rotation_degrees = math.degrees(2.0 * math.acos(min(1.0, abs(dot))))
    return TrackerDelta(
        translation_mm=translation_mm,
        rotation_degrees=rotation_degrees,
    )


def _lighthouse_position_counts(config_path: Path) -> tuple[int, int] | None:
    try:
        status = load_lighthouse_status(config_path)
    except (RuntimeError, ValueError):
        return None
    return status.configured_count, len(status.lighthouses)


def _lighthouse_warning(config_path: Path) -> str | None:
    """Describe an explicitly unsolved lighthouse scene in libsurvive's config."""
    if counts := _lighthouse_position_counts(config_path):
        configured, total = counts
        if configured == total:
            return None
        return (
            "WARNING: libsurvive has world poses for "
            f"{configured}/{total} lighthouses (PositionSet). "
            "Absolute and translational tracking are not trustworthy yet."
        )
    return None


def _format_report(report: TrackerReport, reference: TrackerPose | None = None) -> str:
    pose = report.pose
    label = report.role or "tracker"
    position = " ".join(f"{value:+.4f}" for value in pose.position)
    quaternion = " ".join(f"{value:+.4f}" for value in pose.quaternion_wxyz)
    formatted = (
        f"{label:7} {pose.serial} ({pose.name}) rate={report.rate_hz:6.1f}Hz "
        f"age={report.age_seconds * 1000:5.1f}ms pos=[{position}] "
        f"quat_wxyz=[{quaternion}]"
    )
    if reference is not None:
        delta = tracker_delta(pose, reference)
        translation = " ".join(f"{value:+7.1f}" for value in delta.translation_mm)
        formatted += f" dxyz_mm=[{translation}] drot={delta.rotation_degrees:6.1f}deg"
    return formatted


def _format_separation(reports: tuple[TrackerReport, ...]) -> str | None:
    if len(reports) != 2:
        return None
    first, second = reports
    separation_mm = 1000.0 * math.sqrt(
        sum(
            (left - right) ** 2
            for left, right in zip(
                first.pose.position,
                second.pose.position,
                strict=True,
            )
        )
    )
    return f"tracker separation={separation_mm:.1f}mm"


def run(
    args: Args,
    *,
    source_factory: SourceFactory = _create_source,
    monotonic: Callable[[], float] = time.monotonic,
    emit: Callable[[str], None] = print,
) -> tuple[TrackerReport, ...]:
    """Observe tracker poses, enforce health thresholds, and return a final report."""
    left, right = args.validated_serials()
    monitor = TrackerMonitor(left_serial=left, right_serial=right)
    references: dict[str, TrackerPose] = {}
    started = monotonic()
    deadline = started + args.duration_seconds
    next_print = started + 1.0 / args.print_rate_hz

    if warning := _lighthouse_warning(args.resolved_config_path()):
        emit(warning)
    emit("Relative motion is measured from each tracker's first pose (dxyz_mm, drot).")

    with closing(source_factory(args)) as source:
        while monotonic() < deadline:
            pose = source.next_pose()
            now = monotonic()
            if pose is not None and monitor.observe(pose, received_at=now):
                references.setdefault(pose.serial.casefold(), pose)
            if now >= next_print:
                reports = monitor.reports(now=now)
                for report in reports:
                    emit(_format_report(report, references[report.pose.serial.casefold()]))
                if separation := _format_separation(reports):
                    emit(separation)
                next_print = now + 1.0 / args.print_rate_hz

    finished = monotonic()
    if counts := _lighthouse_position_counts(args.resolved_config_path()):
        configured, total = counts
        emit(f"Lighthouse world poses configured: {configured}/{total}.")

    errors = monitor.health_errors(
        now=finished,
        expected_count=2,
        stale_after_seconds=args.stale_after_seconds,
        min_rate_hz=args.min_rate_hz,
    )
    if errors:
        raise RuntimeError("Tracker validation failed:\n" + "\n".join(errors))

    reports = monitor.reports(now=finished)
    emit("Pose streaming verified:")
    for report in reports:
        emit(_format_report(report, references[report.pose.serial.casefold()]))
    if separation := _format_separation(reports):
        emit(separation)
    return reports


def main() -> None:
    """CLI entry point."""
    try:
        run(tyro.cli(Args))
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()
