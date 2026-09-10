"""Foil model: deflected thrust direction, pressure points, turning loss, and their use by the
effectiveness matrix, the PX4 params export and the sweep."""

from __future__ import annotations

import json

import numpy as np
import pytest
from fastapi.testclient import TestClient

from tiltlab.api.app import app
from tiltlab.core.geometry import rotors_from_scenario
from tiltlab.core.metrics import compute_metrics
from tiltlab.core.sweep import SweepSpec, apply_deflections, candidate_deflections, run_sweep
from tiltlab.export.params import ca_geometry_params
from tiltlab.scenario import Foil, Scenario, deflect_axis

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


@pytest.fixture(scope="module")
def cad() -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))


def test_deflect_axis_horizontal_motor():
    motor = (1.0, 0.0, 0.0)  # blowing aft, thrust forward
    assert np.allclose(deflect_axis(motor, 0), [1, 0, 0])
    assert np.allclose(deflect_axis(motor, 45), [0.7071068, 0, -0.7071068])  # up and forward
    assert np.allclose(deflect_axis(motor, 90), [0, 0, -1])  # pure lift
    assert np.allclose(deflect_axis(motor, 135), [-0.7071068, 0, -0.7071068])  # up and aft
    assert np.allclose(deflect_axis(motor, 180), [-1, 0, 0])
    for d in (0, 30, 45, 90, 120, 180):
        assert np.linalg.norm(deflect_axis(motor, d)) == pytest.approx(1.0)


def test_cad_scenario_has_two_foils_with_horizontal_motors(cad):
    assert [f.id for f in cad.foils] == ["left", "right"]
    assert sorted(i for f in cad.foils for i in f.fan_ids) == list(range(8))
    for foil in cad.foils:
        assert foil.deflection_deg == 45.0
        assert set(foil.pressure_points_frd_m) == set(foil.fan_ids)
    for fan in cad.fans_sorted()[:8]:
        assert fan.tilt_deg == 90.0 and fan.azimuth_deg == 0.0  # motor axis forward, exhaust aft
        pp = cad.effective_pos(fan)
        assert pp[0] < fan.pos_frd_m[0]  # the force acts aft of the motor, in the foil
        assert abs(pp[1] - fan.pos_frd_m[1]) < 0.02
        assert np.allclose(cad.effective_axis(fan), [0.7071068, 0, -0.7071068], atol=1e-6)
    for fan in cad.fans_sorted()[8:]:
        assert cad.foil_for(fan.id) is None and np.allclose(cad.effective_axis(fan), [0, 0, -1])


def test_rotors_and_params_use_the_foil_geometry(cad):
    rotors = rotors_from_scenario(cad)
    for fan, r in zip(cad.fans_sorted()[:8], rotors[:8], strict=True):
        assert np.allclose(r.axis, [0.7071068, 0, -0.7071068], atol=1e-6)
        assert r.position[0] == pytest.approx(cad.effective_pos(fan)[0] - cad.mass.cg_frd_m[0])
    params = ca_geometry_params(cad)
    assert params["CA_ROTOR0_AX"] == pytest.approx(0.7071068, abs=1e-6)
    assert params["CA_ROTOR0_AZ"] == pytest.approx(-0.7071068, abs=1e-6)
    assert params["CA_ROTOR0_PX"] == pytest.approx(rotors[0].position[0], abs=1e-6)
    # deflection changes the exported axis
    sc = apply_deflections(cad, {i: 90.0 for i in range(8)})
    p2 = ca_geometry_params(sc)
    assert p2["CA_ROTOR0_AX"] == pytest.approx(0.0, abs=1e-6) and p2[
        "CA_ROTOR0_AZ"
    ] == pytest.approx(-1.0)


def test_turning_loss_scales_ct(cad):
    """The simple sin^2 loss model applies when no Coanda model is attached."""
    foils = [
        f.model_copy(update={"loss_at_90deg": 0.2, "deflection_deg": 90.0, "coanda": None})
        for f in cad.foils
    ]
    sc = cad.model_copy(update={"foils": foils})
    fan0 = sc.fans_sorted()[0]
    assert sc.foil_ct_scale(fan0) == pytest.approx(0.8)
    assert sc.fan_ct_effective(fan0) == pytest.approx(0.8 * sc.fan_ct(fan0))
    assert ca_geometry_params(sc)["CA_ROTOR0_CT"] == pytest.approx(0.8 * sc.fan_ct(fan0), rel=1e-6)
    sc45 = cad.model_copy(
        update={
            "foils": [
                f.model_copy(update={"loss_at_90deg": 0.2, "coanda": None}) for f in cad.foils
            ]
        }
    )
    assert sc45.foil_ct_scale(fan0) == pytest.approx(1 - 0.2 * 0.5)


def test_foil_validation(cad):
    with pytest.raises(ValueError):
        Foil(id="x", fan_ids=[0], per_fan_deflection_deg={3: 10.0})
    with pytest.raises(ValueError):
        cad.model_validate(
            {
                **cad.model_dump(),
                "foils": [{"id": "a", "fan_ids": [0]}, {"id": "b", "fan_ids": [0]}],
            }
        )


def test_foil_candidates_and_groupings(cad):
    same = list(candidate_deflections(cad, SweepSpec(tilts_deg=[0, 90])))
    assert len(same) == 2 and set(same[1].values()) == {90.0}
    lr = list(candidate_deflections(cad, SweepSpec(tilts_deg=[0, 90], foil_grouping="left_right")))
    assert len(lr) == 4
    left_ids = [f.id for f in cad.fans if f.id < 8 and f.pos_frd_m[1] < 0]
    assert all(lr[1][i] == 0.0 for i in left_ids) and all(
        lr[1][i] == 90.0 for i in range(8) if i not in left_ids
    )
    pp = list(candidate_deflections(cad, SweepSpec(tilts_deg=[0, 90], foil_grouping="per_pair")))
    assert len(pp) == 16 and pp[1][6] == pp[1][7] == 90.0 and pp[1][0] == 0.0


def test_apply_deflections_collapses_uniform(cad):
    sc = apply_deflections(cad, {i: 60.0 for i in range(8)})
    assert all(f.deflection_deg == 60.0 and not f.per_fan_deflection_deg for f in sc.foils)
    sc2 = apply_deflections(cad, {0: 10.0, 1: 10.0})
    assert sc2.foils[0].per_fan_deflection_deg or sc2.foils[1].per_fan_deflection_deg


def test_foil_sweep_physics(cad):
    # same deflection on both foils: all wing thrust points the same way, no level trim except
    # straight down (90)
    res = run_sweep(cad, SweepSpec(tilts_deg=[45, 90, 135], min_headroom=0.0))
    assert res["variable"] == "foil"
    by = {c["pair_tilts_deg"][0]: c for c in res["candidates"]}
    assert not by[45.0]["feasible"] and any(
        "no level-attitude hover trim" in r for r in by[45.0]["reasons"]
    )
    assert not by[135.0]["feasible"]
    assert by[90.0]["fz_up_N"] is not None  # straight down: hover trims, yaw is missing
    assert "yaw unattainable" in by[90.0]["reasons"]
    # opposite left/right deflections cancel Fx but leave a net yaw moment: no hover trim either
    res_lr = run_sweep(
        cad, SweepSpec(tilts_deg=[45, 135], foil_grouping="left_right", min_headroom=0.0)
    )
    assert res_lr["n_feasible"] == 0
    # a segmented foil with fore-aft alternation along each side trims and gives yaw
    res2 = run_sweep(
        cad, SweepSpec(tilts_deg=[45, 135], foil_grouping="per_pair", min_headroom=0.0)
    )
    feas = [c for c in res2["candidates"] if c["feasible"]]
    assert feas, [c["reasons"] for c in res2["candidates"]]
    assert all(len(set(c["pair_tilts_deg"])) > 1 for c in feas)
    assert feas[0]["yaw_Nm"] > 1.0


def test_metrics_use_pressure_points(cad):
    m = compute_metrics(cad, "stock")
    assert m["effectiveness"]["ct_N"][0] == pytest.approx(cad.fan_ct(cad.fans_sorted()[0]))


def test_sweep_endpoint_foil(cad):
    client = TestClient(app)
    r = client.post(
        "/api/sweep",
        json={"scenario": cad.model_dump(mode="json"), "tilts_deg": [90], "min_headroom": 0.0},
    )
    assert r.status_code == 200, r.text
    data = r.json()
    assert data["variable"] == "foil" and data["candidates"][0]["deflections_deg"]["0"] == 90.0
