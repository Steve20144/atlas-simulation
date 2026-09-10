"""M9 export tests: naming, PX4 .params merge onto a full backup, round trip, CSV."""

from __future__ import annotations

import csv
import json
import re
from datetime import datetime
from pathlib import Path

import pytest

from tests.conftest import FIXTURES
from tiltlab.core.params_px4 import read_params_file
from tiltlab.export import (
    export_metrics_csv,
    export_params,
    export_scenario_json,
    import_params_to_scenario,
    latest_backup_params,
    timestamped_name,
)
from tiltlab.scenario import Scenario

REPO = Path(__file__).resolve().parents[1]
BASE = FIXTURES / "20260909_1515_params_v4_rig_airmode.params"
NOW = datetime(2026, 9, 9, 16, 5, 7)
ROTOR_NAMES = {f"CA_ROTOR{i}_{k}" for i in range(10) for k in "PX PY PZ AX AY AZ CT KM".split()}


@pytest.fixture
def scenario() -> Scenario:
    path = REPO / "scenarios" / "baseline_dihedral30.json"
    return Scenario.model_validate(json.loads(path.read_text(encoding="utf-8")))


def _lines(path: Path) -> list[bytes]:
    data = path.read_bytes()
    assert b"\r\n" in data and b"\n" not in data.replace(b"\r\n", b"")
    return data.split(b"\r\n")


def _name(line: bytes) -> str:
    return line.split(b"\t")[2].decode()


def test_timestamped_name_format() -> None:
    assert timestamped_name("baseline_px4", "params", NOW) == "20260909_1605_baseline_px4.params"
    assert timestamped_name("m", ".csv", NOW) == "20260909_1605_m.csv"
    assert re.fullmatch(r"\d{8}_\d{4}_x\.json", timestamped_name("x", "json"))


def test_latest_backup_is_the_stamped_v4_file() -> None:
    assert latest_backup_params() == BASE


@pytest.mark.parametrize(
    ("concept", "extra"),
    [("stock", set()), ("fully_actuated", {"CA_METHOD", "FD_FAIL_P", "FD_FAIL_R"})],
)
def test_params_export_changes_only_geometry_and_concept_lines(
    scenario: Scenario, tmp_path: Path, concept: str, extra: set[str]
) -> None:
    out = export_params(scenario, concept, BASE, tmp_path, now=NOW)
    assert out.name == f"20260909_1605_baseline_dihedral30_{concept}_px4.params"
    base_lines, out_lines = _lines(BASE), _lines(out)
    base_header = [ln for ln in base_lines if ln.startswith(b"#")]
    out_header = [ln for ln in out_lines if ln.startswith(b"#")]
    # every original header line survives, in order, and the tiltlab block is added
    it = iter(out_header)
    assert all(any(h == o for o in it) for h in base_header)
    joined = b"\n".join(out_header).decode()
    assert "tiltlab export" in joined and "xfly80_3280" in joined and "ESTIMATED" in joined
    assert re.search(r"#\s+0\s+30\.000\s+90\.000", joined)
    base_data = [ln for ln in base_lines if ln and not ln.startswith(b"#")]
    out_data = [ln for ln in out_lines if ln and not ln.startswith(b"#")]
    assert len(base_data) == len(out_data)
    changed = {_name(b) for b, o in zip(base_data, out_data, strict=True) if b != o}
    unchanged = [b for b, o in zip(base_data, out_data, strict=True) if b == o]
    assert changed <= ROTOR_NAMES | extra
    expected = {"CA_ROTOR0_AZ", "CA_ROTOR0_AY", "CA_ROTOR0_CT", "CA_ROTOR0_KM", "CA_ROTOR9_CT"}
    assert expected <= changed
    assert extra <= changed
    if concept == "stock":
        assert not {"CA_METHOD", "FD_FAIL_P", "FD_FAIL_R"} & changed
    assert len(unchanged) >= len(base_data) - len(ROTOR_NAMES) - len(extra)
    exported = read_params_file(out).to_dict()
    assert exported["CA_ROTOR_COUNT"] == 10 and exported["CA_ROTOR0_CT"] == pytest.approx(33.3)
    if concept == "fully_actuated":
        assert exported["CA_METHOD"] == 0
        assert exported["FD_FAIL_P"] == 0 and exported["FD_FAIL_R"] == 0
    else:
        assert exported["CA_METHOD"] == 2 and exported["FD_FAIL_P"] == 60


def test_export_import_round_trip_geometry(scenario: Scenario, tmp_path: Path) -> None:
    out = export_params(scenario, "stock", BASE, tmp_path, now=NOW)
    back = import_params_to_scenario(out, scenario)
    for a, b in zip(scenario.fans_sorted(), back.fans_sorted(), strict=True):
        assert a.id == b.id
        assert b.tilt_deg == pytest.approx(a.tilt_deg, abs=1e-4)
        assert b.azimuth_deg == pytest.approx(a.azimuth_deg, abs=1e-4)
        assert b.pos_frd_m == pytest.approx(a.pos_frd_m, abs=1e-6)
        assert b.km == pytest.approx(a.km)
    assert back.control.reaction_torque == scenario.control.reaction_torque
    assert back.control.px4_params_override == {}


def test_metrics_csv_header_and_rows(tmp_path: Path) -> None:
    rows = [
        {"scenario": "A", "concept": "stock", "hover": {"cmd_mean": 0.41, "power_W": 1200.0}},
        {"scenario": "A", "concept": "fully_actuated", "hover": {"cmd_mean": 0.44}, "yaw": 0.2},
        {"scenario": "B", "concept": "stock", "hover": {"cmd_mean": 0.39, "power_W": 1100.0}},
    ]
    path = export_metrics_csv(rows, tmp_path, "metrics", now=NOW)
    assert path.name == "20260909_1605_metrics.csv"
    with path.open(newline="", encoding="utf-8") as fh:
        table = list(csv.reader(fh))
    assert table[0] == ["scenario", "concept", "hover.cmd_mean", "hover.power_W", "yaw"]
    assert len(table) == 1 + len(rows)
    assert table[2][3] == "" and table[2][4] == "0.2"


def test_scenario_json_export(scenario: Scenario, tmp_path: Path) -> None:
    path = export_scenario_json(scenario, tmp_path, now=NOW)
    assert path.name == "20260909_1605_baseline_dihedral30.json"
    again = Scenario.model_validate(json.loads(path.read_text(encoding="utf-8")))
    assert again == scenario
