"""Structured, crash-readable telemetry for teleoperation debugging."""

from __future__ import annotations

import json
import os
import time
from dataclasses import fields, is_dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Protocol

import numpy as np


def _jsonable(value: Any) -> Any:
    if is_dataclass(value) and not isinstance(value, type):
        return {field.name: _jsonable(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, BaseException):
        return {"type": type(value).__name__, "message": str(value)}
    return value


class DebugRecorder(Protocol):
    path: Path | None

    def event(self, event_type: str, **values: Any) -> None: ...

    def close(self) -> None: ...


class NullDebugRecorder:
    """Zero-overhead recorder used for normal runs."""

    path: Path | None = None

    def event(self, _event_type: str, **_values: Any) -> None:
        pass

    def close(self) -> None:
        pass


class JsonlDebugRecorder:
    """Append one self-contained JSON record per event and flush immediately."""

    def __init__(self, path: Path) -> None:
        self.path = path.expanduser()
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        descriptor = os.open(
            self.path,
            os.O_CREAT | os.O_WRONLY | os.O_TRUNC,
            0o600,
        )
        self._output = os.fdopen(descriptor, "w", encoding="utf-8")
        self._closed = False

    def event(self, event_type: str, **values: Any) -> None:
        if self._closed:
            raise RuntimeError("debug recorder is closed")
        record = {
            "event": event_type,
            "time_utc": datetime.now(UTC).isoformat(),
            "monotonic_seconds": time.monotonic(),
            **{name: _jsonable(value) for name, value in values.items()},
        }
        json.dump(record, self._output, allow_nan=False, separators=(",", ":"))
        self._output.write("\n")
        self._output.flush()

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._output.close()


def timestamped_debug_path(root: Path) -> Path:
    """Return a collision-resistant cache path for one teleoperation run."""
    timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
    return root / "logs" / f"teleop-{timestamp}-{os.getpid()}.jsonl"
