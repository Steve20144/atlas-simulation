"""Coanda attachment model and the Gazebo harness export."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from fastapi.testclient import TestClient

from tiltlab.api.app import app
from tiltlab.core.sweep import SweepSpec, apply_deflections, run_sweep
from tiltlab.export.angle_sheet import foil_angle_sheet
from tiltlab.export.gazebo import axis_to_rpy, export_gazebo, frd_to_flu
from tiltlab.export.params import ca_geometry_params
from tiltlab.scenario import Coanda, Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


@pytest.fixture(scope="module")
def cad() -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))


def test_coanda_separation_and_losses():
    c = Coanda(radius_m=0.25, jet_thickness_m=0.08)
    sep = c.separation_deg()
    assert sep == pytest.approx(245 * math.exp(-1.64 * 0.32), rel=1e-6)
    assert 140 < sep < 150
    assert c.turning(45) == (45.0, True)
    eff, attached = c.turning(170)
    assert not attached and eff == pytest.approx(sep)
    assert c.thrust_scale(0) == 1.0
    assert c.thrust_scale(90) == pytest.approx(0.9)
    assert c.thrust_scale(170) == pytest.approx((1 - 0.1 * sep / 90) * 0.7)
    thin = Coanda(radius_m=0.5, jet_thickness_m=0.02)
    assert thin.separation_deg() > sep  # thinner jet on a bigger radius stays attached longer


def test_cad_scenario_uses_coanda(cad):
    for foil in cad.foils:
        assert foil.coanda is not None and foil.coanda.enabled
        assert foil.coanda.radius_m == 0.25
    fan0 = cad.fans_sorted()[0]
    assert cad.fan_effective_deflection(fan0) == 45.0 and cad.fan_jet_attached(fan0)
    assert cad.foil_ct_scale(fan0) == pytest.approx(0.95)
    sc = apply_deflections(cad, {i: 170.0 for i in range(8)})
    f0 = sc.fans_sorted()[0]
    assert not sc.fan_jet_attached(f0)
    assert sc.fan_effective_deflection(f0) == pytest.approx(sc.foils[0].coanda.separation_deg())
    # the exported axis follows the effective turning, not the requested wrap
    ax = ca_geometry_params(sc)
    sep = math.radians(sc.foils[0].coanda.separation_deg())
    assert ax["CA_ROTOR0_AX"] == pytest.approx(math.cos(sep), abs=1e-5)


def test_sweep_flags_separation(cad):
    res = run_sweep(cad, SweepSpec(tilts_deg=[90, 170], min_headroom=0.0))
    by = {c["pair_tilts_deg"][0]: c for c in res["candidates"]}
    assert any("separates" in r for r in by[170.0]["reasons"])
    assert not any("separates" in r for r in by[90.0]["reasons"])


def test_sheet_reports_attachment(cad):
    rows = foil_angle_sheet(apply_deflections(cad, {0: 160.0, 1: 160.0}))
    r0 = next(r for r in rows if r["rotor"] == 0)
    assert r0["deflection_deg"] == 160.0 and not r0["jet_attached"]
    assert r0["effective_turning_deg"] == pytest.approx(r0["coanda_limit_deg"], abs=0.1)


def test_frd_to_flu_and_rpy():
    assert frd_to_flu((1, 2, 3)) == (1, -2, -3)
    up = frd_to_flu((0, 0, -1))
    assert axis_to_rpy(up) == pytest.approx((0.0, 0.0, 0.0))
    fwd_up = frd_to_flu((math.sqrt(0.5), 0, -math.sqrt(0.5)))
    r, p, _ = axis_to_rpy(fwd_up)
    assert p == pytest.approx(math.radians(45)) and r == pytest.approx(0.0)


def test_gazebo_export(cad, tmp_path):
    out = export_gazebo(cad, tmp_path)
    root = ET.parse(out["model_sdf"]).getroot()
    model = root.find("model")
    assert model is not None and model.get("name") == cad.meta.name
    links = [link.get("name") for link in model.findall("link")]
    assert links[0] == "base_link" and sum(name.startswith("rotor_") for name in links) == 10
    plugins = model.findall("plugin")
    motors = [p for p in plugins if "MulticopterMotorModel" in (p.get("name") or "")]
    assert len(motors) == 10
    ca = ca_geometry_params(cad)
    mc = float(motors[0].find("motorConstant").text)
    assert mc * 1000.0**2 == pytest.approx(ca["CA_ROTOR0_CT"], rel=1e-6)
    rotor0 = next(link for link in model.findall("link") if link.get("name") == "rotor_0")
    pose = [float(v) for v in rotor0.find("pose").text.split()]
    expected = frd_to_flu(cad.effective_pos(cad.fans_sorted()[0]) - np.asarray(cad.mass.cg_frd_m))
    assert pose[:3] == pytest.approx(expected, abs=1e-4)
    ET.parse(out["world_sdf"])  # well formed
    airframe = open(out["airframe"], encoding="utf-8").read()
    assert (
        f"CA_ROTOR0_AX {ca['CA_ROTOR0_AX']:.6g}" in airframe and "SIM_GZ_EC_FUNC10 110" in airframe
    )
    # PX4's sh sources the airframe line by line; a CRLF file makes every "param set-default" fail
    # silently and PX4 runs on defaults (4 ESC outputs), so the files must be LF even from Windows.
    for key in ("airframe", "model_sdf", "world_sdf"):
        assert b"\r" not in open(out[key], "rb").read(), key
    assert "meshes" in out["mesh"]
    client = TestClient(app)
    r = client.post("/api/export/gazebo", json={"scenario": cad.model_dump(mode="json")})
    assert r.status_code == 200, r.text
