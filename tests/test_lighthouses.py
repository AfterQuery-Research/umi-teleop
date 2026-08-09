import pytest

from umi_teleop_fixed.lighthouses import (
    lighthouse_calibration_id_path,
    load_lighthouse_calibration_id,
    load_lighthouse_status,
    require_lighthouse_calibration,
    save_new_lighthouse_calibration_id,
)


def lighthouse_config(
    *,
    ids=("100", "200"),
    ootx=None,
    positions=None,
):
    ootx = ootx or tuple("1" for _ in ids)
    positions = positions or tuple("1" for _ in ids)
    assert len(ids) == len(ootx) == len(positions)
    blocks = []
    for index in range(len(ids)):
        blocks.append(
            f'''"lighthouse{index}":{{
"id":"{ids[index]}",
"pose":["{index}.0","1.0","2.0","1.0","0.0","0.0","0.0"],
"OOTXSet":"{ootx[index]}",
"PositionSet":"{positions[index]}"
}}'''
        )
    return "\n".join(blocks) + "\n"


def test_loads_complete_lighthouse_status(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(lighthouse_config())

    status = load_lighthouse_status(path)

    assert status.ready(expected_count=2)
    assert status.ootx_count == 2
    assert status.configured_count == 2
    assert status.lighthouses[1].pose[:3] == (1.0, 1.0, 2.0)


def test_require_rejects_incomplete_calibration(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(lighthouse_config(ootx=("1", "0"), positions=("1", "0")))

    with pytest.raises(RuntimeError, match="OOTX received for 1/2"):
        require_lighthouse_calibration(path)


def test_require_rejects_duplicate_lighthouse_ids(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(lighthouse_config(ids=("same", "same")))

    with pytest.raises(RuntimeError, match="IDs must be unique"):
        require_lighthouse_calibration(path)


def test_require_accepts_all_calibrated_lighthouses_beyond_two(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(lighthouse_config(ids=("100", "200", "300")))

    status = require_lighthouse_calibration(path)

    assert len(status.lighthouses) == 3
    assert status.ready(expected_count=3)


def test_require_rejects_fewer_than_two_lighthouses(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(lighthouse_config(ids=("100",)))

    with pytest.raises(RuntimeError, match="expected at least 2 lighthouses"):
        require_lighthouse_calibration(path)


def test_lighthouse_calibration_id_is_private_and_replaced(tmp_path):
    config_path = tmp_path / "config.json"

    first = save_new_lighthouse_calibration_id(config_path)
    second = save_new_lighthouse_calibration_id(config_path)

    assert first != second
    assert load_lighthouse_calibration_id(config_path) == second
    assert lighthouse_calibration_id_path(config_path).stat().st_mode & 0o777 == 0o600
