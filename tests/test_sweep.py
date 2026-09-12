"""Tilt sweep: grid generation, feasibility, ranking, API."""

from __future__ import annotations

import json

import pytest
from fastapi.testclient import TestClient

from tiltlab.api.app import app
from tiltlab.core.sweep import (
    SweepSpec,
    apply_angles,
    candidate_angles,
    nose_candidates,
    nose_fan_ids,
    run_sweep,
    signed_lateral_to_angles,
    sweep_table,
)
from tiltlab.scenario import Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


@pytest.fixture(scope="module")
def scenario() -> Scenario:
    """The CAD scenario as a raw-tilt problem: foils removed, every fan vertical."""
    sc = Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))
    fans = [f.model_copy(update={"tilt_deg": 0.0, "azimuth_deg": 0.0}) for f in sc.fans]
    return sc.model_copy(update={"fans": fans, "foils": []})


def test_grid_shared_and_per_pair(scenario):
    shared = list(
        candidate_angles(scenario, SweepSpec(tilts_deg=[0, 10, 20], azimuth_mode="inward"))
    )
    assert len(shared) == 3
    per_pair = list(
        candidate_angles(
            scenario,
            SweepSpec(
                tilts_deg=[0, 10], per_pair=True, centreline_tilts_deg=[0, 5], azimuth_mode="inward"
            ),
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
    result = run_sweep(
        scenario,
        SweepSpec(tilts_deg=[0, 10], per_pair=True, min_headroom=0.1, azimuth_mode="inward"),
    )
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
    """Opposite fore-aft directions across pairs cancel Fx and give yaw from differential thrust."""
    result = run_sweep(
        scenario, SweepSpec(tilts_deg=[10, 20], azimuth_mode="alternating", min_headroom=0.0)
    )
    assert result["n_feasible"] == 2
    by_tilt = {c["pair_tilts_deg"][0]: c for c in result["candidates"]}
    assert by_tilt[20.0]["yaw_Nm"] > by_tilt[10.0]["yaw_Nm"] > 1.0
    assert by_tilt[20.0]["power_W"] > by_tilt[10.0]["power_W"]  # more tilt, less lift per watt
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


def test_diagnostics_explain_threshold_blocks(scenario):
    res = run_sweep(
        scenario, SweepSpec(tilts_deg=[10, 20], azimuth_mode="alternating", min_headroom=0.9)
    )
    d = res["diagnostics"]
    assert res["n_feasible"] == 0
    assert d["n_controllable"] >= 1 and d["n_blocked_by_thresholds"] == d["n_controllable"]
    assert 0 < d["best_headroom_controllable"] < 0.9 and d["best_yaw_controllable"] > 1.0
    assert d["most_common_reason_near_miss"].startswith("headroom")
    res2 = run_sweep(scenario, SweepSpec(tilts_deg=[45], azimuth_mode="forward"))
    assert res2["diagnostics"]["n_controllable"] == 0
    assert res2["diagnostics"]["most_common_reason_near_miss"]


def test_rank_by_orders_feasible_candidates(scenario):
    spec = dict(tilts_deg=[10, 15, 20], azimuth_mode="alternating", min_headroom=0.0)
    by_power = run_sweep(scenario, SweepSpec(**spec, rank_by="power"))["candidates"]
    by_yaw = run_sweep(scenario, SweepSpec(**spec, rank_by="yaw"))["candidates"]
    feas_p = [c for c in by_power if c["feasible"]]
    feas_y = [c for c in by_yaw if c["feasible"]]
    assert len(feas_p) == len(feas_y) >= 2
    assert [c["power_W"] for c in feas_p] == sorted(c["power_W"] for c in feas_p)
    assert [c["yaw_Nm"] for c in feas_y] == sorted((c["yaw_Nm"] for c in feas_y), reverse=True)
    assert by_power[0]["pair_tilts_deg"][0] == 10.0  # the least tilted geometry hovers cheapest
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[0], rank_by="colour")


def test_spec_validation():
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[190])
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[0], azimuth_mode="sideways")
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[0], foil_grouping="odd")


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


# ---------------------------------------------------------------- nose fans sideways


@pytest.fixture(scope="module")
def foil_scenario() -> Scenario:
    """The CAD scenario as flown: segmented foils on the wing fans, nose fans vertical."""
    return Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))


def test_signed_lateral_tilt_maps_to_left_right_azimuth():
    """Negative is left (-Y, azimuth 270), positive right (+Y, azimuth 90), zero straight down."""
    from tiltlab.scenario import tilt_azimuth_to_axis

    assert signed_lateral_to_angles(0.0) == (0.0, 0.0)
    assert signed_lateral_to_angles(-20.0) == (20.0, 270.0)
    assert signed_lateral_to_angles(20.0) == (20.0, 90.0)
    ax = tilt_azimuth_to_axis(*signed_lateral_to_angles(-30.0))
    assert ax[0] == 0.0 and ax[1] < 0.0 and ax[2] < 0.0  # no fore-aft share, jet to the left
    ax = tilt_azimuth_to_axis(*signed_lateral_to_angles(30.0))
    assert ax[0] == 0.0 and ax[1] > 0.0 and ax[2] < 0.0


def test_nose_pairings(foil_scenario):
    front, rear = nose_fan_ids(foil_scenario)
    fans = {f.id: f for f in foil_scenario.fans}
    assert fans[front].pos_frd_m[0] > fans[rear].pos_frd_m[0]
    opposed = list(
        nose_candidates(foil_scenario, SweepSpec(tilts_deg=[90], nose_tilts_deg=[-20, 0, 20]))
    )
    assert opposed == [
        {front: -20.0, rear: 20.0},
        {front: 0.0, rear: 0.0},
        {front: 20.0, rear: -20.0},
    ]
    same = list(
        nose_candidates(
            foil_scenario, SweepSpec(tilts_deg=[90], nose_tilts_deg=[-20, 20], nose_pairing="same")
        )
    )
    assert same == [{front: -20.0, rear: -20.0}, {front: 20.0, rear: 20.0}]
    indep = list(
        nose_candidates(
            foil_scenario,
            SweepSpec(tilts_deg=[90], nose_tilts_deg=[-20, 20], nose_pairing="independent"),
        )
    )
    assert len(indep) == 4 and indep[1] == {front: -20.0, rear: 20.0}
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[90], nose_tilts_deg=[95])
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[90], nose_tilts_deg=[10], nose_pairing="crossed")


def test_foil_sweep_with_nose_grid_applies_sideways_tilt(foil_scenario):
    """Every foil candidate is evaluated at every nose setting; the record carries the signed
    front/rear tilt and the tilt/azimuth actually applied so the UI can load it."""
    spec = SweepSpec(
        tilts_deg=[45, 135],
        foil_grouping="left_right",
        nose_tilts_deg=[-15, 15],
        min_headroom=0.0,
    )
    result = run_sweep(foil_scenario, spec)
    assert result["variable"] == "foil"
    assert result["n_evaluated"] == 2 * 2 * 2
    assert result["spec"]["nose_tilts_deg"] == [-15.0, 15.0]
    front, rear = nose_fan_ids(foil_scenario)
    rec = next(r for r in result["candidates"] if r["nose_tilts_deg"] == [-15.0, 15.0])
    assert rec["nose_angles_deg"] == {str(front): [15.0, 270.0], str(rear): [15.0, 90.0]}
    assert rec["centreline_tilt_deg"] == 15.0
    # the wing fans keep their foil deflections
    assert set(rec["deflections_deg"]) == {str(i) for i in range(8)}
    # the nose tilt changes the geometry: a tilted nose fan carries less lift, so power differs
    same_foils = [
        r for r in result["candidates"] if r["deflections_deg"] == rec["deflections_deg"]
    ]
    assert len(same_foils) == 2 and same_foils[0]["power_W"] != same_foils[1]["power_W"]
    assert "nose F/R" in sweep_table(result, 3)


def test_tilt_sweep_nose_grid_overrides_centreline(scenario):
    spec = SweepSpec(
        tilts_deg=[0, 10],
        centreline_tilts_deg=[0, 5],
        nose_tilts_deg=[-30, 30],
        nose_pairing="same",
        min_headroom=0.0,
    )
    result = run_sweep(scenario, spec)
    # the centreline grid collapses to one entry when the nose grid is active
    assert result["n_evaluated"] == 2 * 2
    rec = next(r for r in result["candidates"] if r["nose_tilts_deg"] == [30.0, 30.0])
    assert rec["tilts_deg"][8] == 30.0 and rec["azimuths_deg"][8] == 90.0
    assert rec["tilts_deg"][9] == 30.0 and rec["azimuths_deg"][9] == 90.0
    assert rec["nose_angles_deg"] is not None
    plain = run_sweep(scenario, SweepSpec(tilts_deg=[0], min_headroom=0.0))["candidates"][0]
    assert plain["nose_tilts_deg"] is None and plain["nose_angles_deg"] is None


def test_sweep_endpoint_nose(foil_scenario):
    client = TestClient(app)
    body = {
        "scenario": foil_scenario.model_dump(mode="json"),
        "tilts_deg": [90],
        "nose_tilts_deg": [-10, 10],
        "nose_pairing": "opposed",
        "min_headroom": 0.0,
        "top": 5,
    }
    r = client.post("/api/sweep", json=body)
    assert r.status_code == 200, r.text
    data = r.json()
    assert data["n_evaluated"] == 2 and data["spec"]["nose_pairing"] == "opposed"
    assert sorted(c["nose_tilts_deg"] for c in data["candidates"]) == [
        [-10.0, 10.0],
        [10.0, -10.0],
    ]
    r = client.post("/api/sweep", json={**body, "nose_pairing": "crossed"})
    assert r.status_code == 422
