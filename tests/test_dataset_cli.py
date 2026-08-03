from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from umi_teleop.dataset import DatasetRecordingError, dataset_features
from umi_teleop.dataset_cli import UploadConfig, upload_dataset


class UploadFake:
    def __init__(self, *, episodes: int = 2, frames: int = 60) -> None:
        self.meta = type(
            "Meta",
            (),
            {
                "features": dataset_features(),
                "total_episodes": episodes,
                "total_frames": frames,
            },
        )()
        self.pushes: list[dict[str, Any]] = []

    def push_to_hub(self, **kwargs: Any) -> None:
        self.pushes.append(kwargs)


def existing_root(tmp_path: Path) -> Path:
    root = tmp_path / "dataset"
    (root / "meta").mkdir(parents=True)
    (root / "meta" / "info.json").write_text("{}")
    return root


def test_upload_existing_dataset_validates_and_defaults_private(tmp_path: Path) -> None:
    dataset = UploadFake()
    root = existing_root(tmp_path)
    loaded: list[tuple[str, Path]] = []

    def loader(repo_id: str, dataset_root: Path):
        loaded.append((repo_id, dataset_root))
        return dataset

    upload_dataset(UploadConfig("afterquery/retry", root), loader=loader)

    assert loaded == [("afterquery/retry", root)]
    assert dataset.pushes == [{"private": True}]


def test_upload_existing_dataset_rejects_empty_dataset(tmp_path: Path) -> None:
    root = existing_root(tmp_path)
    with pytest.raises(DatasetRecordingError, match="no saved frames"):
        upload_dataset(
            UploadConfig("afterquery/empty", root),
            loader=lambda _repo_id, _root: UploadFake(episodes=0, frames=0),
        )


def test_upload_existing_dataset_keeps_local_error_context(tmp_path: Path) -> None:
    class FailingUpload(UploadFake):
        def push_to_hub(self, **kwargs: Any) -> None:
            raise ConnectionError("offline")

    root = existing_root(tmp_path)
    with pytest.raises(DatasetRecordingError, match="local dataset is unchanged"):
        upload_dataset(
            UploadConfig("afterquery/retry", root),
            loader=lambda _repo_id, _root: FailingUpload(),
        )
