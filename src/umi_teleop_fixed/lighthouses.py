"""Read and validate libsurvive lighthouse scene calibration."""

from __future__ import annotations

import math
import os
import re
import secrets
import tempfile
from dataclasses import dataclass
from pathlib import Path

MIN_LIGHTHOUSES = 2


@dataclass(frozen=True)
class Lighthouse:
    """One base station decoded from libsurvive's config format."""

    index: int
    base_station_id: str
    ootx_set: bool
    position_set: bool
    pose: tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class LighthouseStatus:
    """Calibration state for every base station in one config."""

    lighthouses: tuple[Lighthouse, ...]

    @property
    def ootx_count(self) -> int:
        return sum(lighthouse.ootx_set for lighthouse in self.lighthouses)

    @property
    def configured_count(self) -> int:
        return sum(lighthouse.position_set for lighthouse in self.lighthouses)

    def ready(self, *, expected_count: int) -> bool:
        return (
            len(self.lighthouses) == expected_count
            and self.ootx_count == expected_count
            and self.configured_count == expected_count
        )


def lighthouse_calibration_id_path(config_path: Path) -> Path:
    """Return the generation marker paired with a libsurvive configuration."""
    return config_path.with_name(f"{config_path.stem}.calibration-id")


def save_new_lighthouse_calibration_id(config_path: Path) -> str:
    """Atomically mark a newly solved scene and return its private generation ID."""
    calibration_id = secrets.token_hex(32)
    path = lighthouse_calibration_id_path(config_path)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            output.write(calibration_id + "\n")
            output.flush()
            os.fsync(output.fileno())
        temporary_path.chmod(0o600)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return calibration_id


def load_lighthouse_calibration_id(config_path: Path) -> str:
    """Load the generation ID required to pair a scene with its axis mapping."""
    path = lighthouse_calibration_id_path(config_path)
    try:
        calibration_id = path.read_text(encoding="utf-8").strip().casefold()
    except OSError as error:
        raise RuntimeError(
            "Lighthouse and teleop-frame calibration are not paired; run "
            "'uv run umi-calibrate-tracking'"
        ) from error
    if len(calibration_id) != 64 or any(
        character not in "0123456789abcdef" for character in calibration_id
    ):
        raise RuntimeError(f"invalid Lighthouse calibration ID in {path}")
    return calibration_id


def _field(block: str, name: str) -> str:
    match = re.search(rf'"{re.escape(name)}"\s*:\s*"([^"]*)"', block)
    if match is None:
        raise ValueError(f"lighthouse config is missing {name}")
    return match.group(1)


def load_lighthouse_status(path: Path) -> LighthouseStatus:
    """Parse libsurvive's line-oriented config without treating it as JSON."""
    try:
        contents = path.read_text(encoding="utf-8")
    except OSError as error:
        raise RuntimeError(f"Could not read libsurvive config {path}: {error}") from error

    lighthouses = []
    blocks = re.finditer(r'"lighthouse(\d+)"\s*:\s*\{(.*?)\n\}', contents, re.DOTALL)
    for match in blocks:
        block = match.group(2)
        pose_match = re.search(r'"pose"\s*:\s*\[([^]]*)\]', block)
        if pose_match is None:
            raise ValueError("lighthouse config is missing pose")
        raw_pose = re.findall(r'"([^"]*)"', pose_match.group(1))
        if len(raw_pose) != 7:
            raise ValueError("lighthouse pose must contain seven values")
        try:
            pose = tuple(float(value) for value in raw_pose)
        except ValueError as error:
            raise ValueError("lighthouse pose must be numeric") from error
        if not all(math.isfinite(value) for value in pose):
            raise ValueError("lighthouse pose must be finite")
        lighthouses.append(
            Lighthouse(
                index=int(match.group(1)),
                base_station_id=_field(block, "id"),
                ootx_set=_field(block, "OOTXSet") == "1",
                position_set=_field(block, "PositionSet") == "1",
                pose=pose,  # type: ignore[arg-type]
            )
        )
    return LighthouseStatus(tuple(sorted(lighthouses, key=lambda value: value.index)))


def require_lighthouse_calibration(
    path: Path,
    *,
    expected_count: int | None = None,
) -> LighthouseStatus:
    """Return a valid frozen scene with either an exact or automatically detected count."""
    try:
        status = load_lighthouse_status(path)
    except (RuntimeError, ValueError) as error:
        raise RuntimeError(
            "Lighthouse calibration is not ready. Run 'uv run umi-calibrate-tracking'."
        ) from error
    detected = len(status.lighthouses)
    errors = []
    if expected_count is None and detected < MIN_LIGHTHOUSES:
        errors.append(f"expected at least {MIN_LIGHTHOUSES} lighthouses, found {detected}")
    elif expected_count is not None and detected != expected_count:
        errors.append(f"expected {expected_count} lighthouses, found {detected}")
    required_count = detected if expected_count is None else expected_count
    if status.ootx_count != required_count:
        errors.append(f"OOTX received for {status.ootx_count}/{required_count} lighthouses")
    if status.configured_count != required_count:
        errors.append(
            f"world poses configured for {status.configured_count}/{required_count} lighthouses"
        )
    for lighthouse in status.lighthouses:
        if lighthouse.position_set:
            quaternion_norm = math.sqrt(sum(value * value for value in lighthouse.pose[3:]))
            if quaternion_norm < 0.5:
                errors.append(f"lighthouse {lighthouse.index} has an invalid orientation")
    identifiers = [value.base_station_id for value in status.lighthouses]
    if len(set(identifiers)) != len(identifiers):
        errors.append("lighthouse IDs must be unique")
    if errors:
        raise RuntimeError(
            "Lighthouse calibration is not ready: "
            + "; ".join(errors)
            + ". Run 'uv run umi-calibrate-tracking'."
        )
    return status
