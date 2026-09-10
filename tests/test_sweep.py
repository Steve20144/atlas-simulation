"""Tilt sweep: grid generation, feasibility, ranking, API."""

from __future__ import annotations

import json

import pytest
from fastapi.testclient import TestClient

from tiltlab.api.app import app
from tiltlab.core.sweep import SweepSpec, apply_angles, candidate_angles, run_sweep, sweep_table
from tiltlab.scenario import Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


@pytest.fixture(scope="module")
def scenario() -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))


def test_grid_shared_and_per_pair(scenario):
    shared = list(candidate_angles(scenario, SweepSpec(tilts_deg=[0, 10, 20])))
    assert len(shared) == 3
    per_pair = list(
        candidate_angles(
            scenario, SweepSpec(tilts_deg=[0, 10], per_pair=True, centreline_tilts_deg=[0, 5])
        )
    )
    assert len(per_pair) == 2**4 * 2
    # inward: the left fan (negative Y) gets azimuth 90, the right fan 270
    angles = shared[1]
    left = next(f for f in scenario.fans if f.id in (0, 1) and f.pos_frd_m[1] < 0)
    right = next(f for f in scenario.fans if f.id in (0, 1) and f.pos_frd_m[1] > 0)
    assert angles[left.id] == (10.0, 90.0) and angles[right.id] == (10.0, 270.0)
    assert angles[8] == (0.0, 0.0)


def test_apply_angles_only_touches_orientation(scenario):
    angles = next(candidate_angles(scenario, SweepSpec(tilts_deg=[25])))
    sc = apply_angles(scenario, angles)
    assert [f.tilt_deg for f in sc.fans][:8] == [25.0] * 8
    assert [f.pos_frd_m for f in sc.fans] == [f.pos_frd_m for f in scenario.fans]
    assert sc.mass == scenario.mass


def test_shared_tilt_on_a_straight_fan_line_cannot_separate_roll_from_yaw(scenario):
    """The eight wing fans lie on a straight line, so one shared tilt makes the roll, yaw and
    Fy rows linearly dependent (rank 4) and no shared-tilt geometry is feasible under stock PX4."""
    result = run_sweep(scenario, SweepSpec(tilts_deg=[0, 15, 30, 45], min_headroom=0.0))
    assert result["n_evaluated"] == 4 and result["n_feasible"] == 0
    reasons = {r for c in result["candidates"] for r in c["reasons"]}
    assert any("yaw unattainable" in r for r in reasons)
    assert "pair tilts deg" in sweep_table(result)


def test_per_pair_sweep_ranks_feasible_by_power(scenario):
    result = run_sweep(scenario, SweepSpec(tilts_deg=[0, 10], per_pair=True, min_headroom=0.1))
    assert result["n_evaluated"] == 16
    feas = [c for c in result["candidates"] if c["feasible"]]
    assert feas and feas == result["candidates"][: len(feas)]
    powers = [c["power_W"] for c in feas]
    assert powers == sorted(powers)
    best = result["best"]
    assert best is not None and set(best) >= {"tilts_deg", "azimuths_deg", "yaw_Nm", "reasons"}
    assert best["yaw_Nm"] is not None and best["yaw_Nm"] > 0
    # alternating tilts break the dependence; a uniform tilt does not
    assert len(set(best["pair_tilts_deg"])) > 1


def test_infeasible_reasons_reported(scenario):
    result = run_sweep(scenario, SweepSpec(tilts_deg=[0], min_headroom=0.99))
    c = result["candidates"][0]
    assert not c["feasible"] and any("headroom" in r for r in c["reasons"])
    assert result["best"] is None


def test_alternating_fore_aft_gives_yaw_under_stock_px4(scenario):
    """Opposite fore-aft directions across pairs cancel Fx and yield yaw from differential thrust."""
    result = run_sweep(
        scenario, SweepSpec(tilts_deg=[15, 45], azimuth_mode="alternating", min_headroom=0.05)
    )
    assert result["n_feasible"] == 2
    by_tilt = {c["pair_tilts_deg"][0]: c for c in result["candidates"]}
    assert by_tilt[45.0]["yaw_Nm"] > by_tilt[15.0]["yaw_Nm"] > 1.0
    assert by_tilt[45.0]["power_W"] > by_tilt[15.0]["power_W"]
    angles = list(
        candidate_angles(scenario, SweepSpec(tilts_deg=[15], azimuth_mode="alternating"))
    )[0]
    assert angles[0][1] == 0.0 and angles[2][1] == 180.0  # outer pair forward, next pair aft


def test_same_direction_forward_has_no_level_trim(scenario):
    result = run_sweep(
        scenario, SweepSpec(tilts_deg=[45], azimuth_mode="forward", min_headroom=0.0)
    )
    c = result["candidates"][0]
    assert not c["feasible"] and any("no level-attitude hover trim" in r for r in c["reasons"])


def test_spec_validation():
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[95])
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[0], azimuth_mode="sideways")


def test_sweep_endpoint(scenario):
    client = TestClient(app)
    body = {
        "scenario": scenario.model_dump(mode="json"),
        "tilts_deg": [0, 20],
        "min_headroom": 0.0,
        "top": 1,
    }
    r = client.post("/api/sweep", json=body)
    assert r.status_code == 200, r.text
    data = r.json()
    assert data["n_evaluated"] == 2 and len(data["candidates"]) == 1
    assert data["spec"]["azimuth_mode"] == "forward"
