"""Control-authority checks (metrics["control"]) and their use in the sweep."""

from __future__ import annotations

import json

import numpy as np
import pytest

from tiltlab.core.metrics import ControlRequirements, compute_metrics, fan_lag_s, inertia_matrix
from tiltlab.core.sweep import SweepSpec, run_sweep
from tiltlab.scenario import Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


def load(name: str) -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / f"{name}.json").read_text()))


@pytest.fixture(scope="module")
def hover() -> Scenario:
    return load("atlas_phase01_cad_hover")


def test_control_block_is_torque_over_inertia(hover):
    m = compute_metrics(hover)
    c = m["control"]
    inertia, placeholder = inertia_matrix(hover)
    assert placeholder is True and c["inertia_placeholder"] is True
    for k, axis in enumerate(("roll", "pitch", "yaw")):
        a = c["axes"][axis]
        auth = m["authority"][axis]
        tau = min(auth["plus"], auth["minus"])
        assert a["torque_Nm"] == pytest.approx(tau)
        assert a["accel_rad_s2"] == pytest.approx(tau / inertia[k, k])
        # time to 10 deg from rest under the full attainable torque
        assert a["time_to_10deg_s"] == pytest.approx(
            np.sqrt(2 * np.radians(10) / a["accel_rad_s2"])
        )
        # the linear region cannot exceed the LP authority
        assert 0.0 <= a["linear_fraction"] <= 1.0 + 1e-9
        assert 0.0 <= a["surge_leak_frac_of_weight"] < 1.0
    assert c["fan_lag_s"] == pytest.approx(fan_lag_s(hover))
    assert c["rate_bandwidth_rad_s"] == pytest.approx(min(4.0, 1 / (2.5 * c["fan_lag_s"])))
    names = {chk["name"] for chk in c["checks"]}
    assert {
        "roll acceleration",
        "pitch acceleration",
        "yaw acceleration",
        "off-axis coupling",
    } <= names
    assert c["weakest_axis"] in ("roll", "pitch", "yaw")
    assert c["score"] > 0.0


def test_requirements_drive_checks_and_score(hover):
    easy = compute_metrics(hover, requirements=ControlRequirements(1.0, 1.0, 1.0, 1.0, 1.0))
    hard = compute_metrics(hover, requirements=ControlRequirements(100.0, 100.0, 100.0, 0.3, 0.1))
    assert easy["control"]["pass"] is True
    assert hard["control"]["pass"] is False
    assert easy["control"]["score"] > hard["control"]["score"]
    failed = [chk["name"] for chk in hard["control"]["checks"] if not chk["pass"]]
    assert "roll acceleration" in failed and "pitch acceleration" in failed


def test_unattainable_axes_have_zero_acceleration():
    m = compute_metrics(load("atlas_phase01_cad"))  # all-45-degree set: no level hover trim
    for axis in ("roll", "pitch", "yaw"):
        a = m["control"]["axes"][axis]
        assert (
            a["attainable"] is False and a["accel_rad_s2"] == 0.0 and a["time_to_10deg_s"] is None
        )
    assert m["control"]["score"] == 0.0


def test_sweep_filters_and_ranks_by_control(hover):
    base = load("atlas_phase01_cad")
    grid = [60.0, 120.0, 135.0]
    spec = SweepSpec(
        tilts_deg=grid,
        variable="foil",
        foil_grouping="per_pair",
        min_headroom=0.0,
        min_pitch_accel=8.0,
        min_yaw_accel=2.5,
        max_coupling=0.3,
        rank_by="control",
    )
    res = run_sweep(base, spec)
    feasible = [r for r in res["candidates"] if r["feasible"]]
    assert feasible, "some per-pair set must pass"
    scores = [r["control_score"] for r in feasible]
    assert scores == sorted(scores, reverse=True)
    for r in feasible:
        assert r["pitch_acc"] >= 8.0 and r["yaw_acc"] >= 2.5 and r["coupling_max"] <= 0.3
    assert res["spec"]["min_pitch_accel"] == 8.0 and res["objective"].startswith("control score")
    # a threshold nobody meets is reported as a threshold miss, not as an uncontrollable geometry
    strict = run_sweep(
        base,
        SweepSpec(
            tilts_deg=grid,
            variable="foil",
            foil_grouping="per_pair",
            min_headroom=0.0,
            min_roll_accel=1000.0,
            rank_by="control",
        ),
    )
    assert strict["n_feasible"] == 0
    assert strict["diagnostics"]["n_controllable"] > 0
    assert any(
        "roll acceleration" in reason for r in strict["candidates"] for reason in r["reasons"]
    )


def test_spec_rejects_bad_control_thresholds():
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[90.0], min_roll_accel=-1.0)
    with pytest.raises(ValueError):
        SweepSpec(tilts_deg=[90.0], max_coupling=1.5)


def test_hover_pitch_rotates_geometry_into_the_flight_controller_frame():
    """Hovering nose-up by t puts the wing jets of a foil at deflection d where a level hover
    would put them at d + t; the sweep evaluates every geometry at each requested attitude."""
    level = load("atlas_phase01_cad")  # foils at 45 deg, jets 45 deg forward of vertical
    pitched = level.model_copy(deep=True)
    pitched.frame.hover_pitch_deg = 45.0
    ax = pitched.hover_axis(pitched.fans_sorted()[0])
    assert ax[0] == pytest.approx(0.0, abs=1e-6) and ax[2] == pytest.approx(-1.0, abs=1e-6)
    nose = pitched.hover_axis(next(f for f in pitched.fans if f.id == 8))
    assert nose[0] == pytest.approx(-np.sin(np.radians(45)), abs=1e-6)  # vertical fan now aft-up
    assert not compute_metrics(level)["hover"]["exact"]
    # the as-built set trims at no attitude: wing and nose fans always disagree in direction
    assert not compute_metrics(pitched)["hover"]["exact"]
    res = run_sweep(
        level,
        SweepSpec(
            tilts_deg=[45.0, 135.0],
            variable="foil",
            foil_grouping="per_pair",
            min_headroom=0.0,
            hover_pitch_deg=[0.0, 15.0],
        ),
    )
    cands = res["candidates"]
    assert {r["hover_pitch_deg"] for r in cands} == {0.0, 15.0} and len(cands) == 32
    assert any(r["feasible"] and r["hover_pitch_deg"] == 15.0 for r in cands)
    assert res["spec"]["hover_pitch_deg"] == [0.0, 15.0]
