#!/usr/bin/env python3
"""Validate and upload an existing local UMI LeRobot dataset without robot access."""

from __future__ import annotations

import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import tyro

from umi_teleop.dataset import DatasetRecordingError, validate_finalized_dataset


class UploadDataset(Protocol):
    meta: Any

    def push_to_hub(self, **kwargs: Any) -> None: ...


DatasetLoader = Callable[[str, Path], UploadDataset]


@dataclass(frozen=True)
class UploadConfig:
    """Existing local LeRobot dataset and Hub destination."""

    repo_id: str
    root: Path
    private: bool = True


def _load_dataset(repo_id: str, root: Path) -> UploadDataset:
    try:
        from lerobot.datasets import LeRobotDataset
    except ImportError as exc:
        raise DatasetRecordingError(
            "LeRobot upload dependencies are unavailable; install the dataset extra"
        ) from exc
    try:
        return LeRobotDataset(repo_id, root=root, download_videos=False)
    except Exception as exc:
        raise DatasetRecordingError(f"failed to open local dataset: {exc}") from exc


def upload_dataset(config: UploadConfig, loader: DatasetLoader = _load_dataset) -> None:
    repo_id = config.repo_id.strip()
    if repo_id.count("/") != 1 or any(not part for part in repo_id.split("/")):
        raise DatasetRecordingError("dataset repo ID must be OWNER/NAME")
    if not (config.root / "meta" / "info.json").is_file():
        raise DatasetRecordingError(f"dataset root is missing meta/info.json: {config.root}")

    dataset = loader(repo_id, config.root)
    validate_finalized_dataset(dataset)
    try:
        dataset.push_to_hub(private=config.private)
    except Exception as exc:
        raise DatasetRecordingError(
            f"Hub upload failed; the local dataset is unchanged: {exc}"
        ) from exc
    print(
        f"DATASET uploaded repo={repo_id} episodes={dataset.meta.total_episodes} "
        f"frames={dataset.meta.total_frames} private={config.private}",
        flush=True,
    )


def main() -> None:
    try:
        upload_dataset(tyro.cli(UploadConfig))
    except DatasetRecordingError as exc:
        print(f"DATASET ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()
