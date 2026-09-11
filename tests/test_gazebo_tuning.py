"""Gazebo harness: controller sizing, thrust linearisation, fan lag and mass bookkeeping."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET

import pytest

from tiltlab.export.gazebo import (
    ROTOR_MASS_KG,
    airframe_id,
    body_mass_kg,
    export_gazebo,
    fan_lag_s,
    px4_tuning,
)
from tiltlab.scenario import Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


@pytest.fixture(scope="module")
def hover() -> Scenario:
    return Scenario.model_validate(
        json.loads((SCENARIOS / "atlas_phase01_cad_hover.json").read_text())
    )


@pytest.fixture(scope="module")
def cad() -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))


def test_tuning_sized_from_authority_and_lag(hover):
    t = px4_tuning(hover)
    assert t["THR_MDL_FAC"] == 1.0
    assert 0.3 < t["MPC_THR_HOVER"] < 0.6  # tiltlab hover collective for the per-pair set
    lag = fan_lag_s(hover)
    assert lag == pytest.approx(0.15)
    w_c = min(4.0, 1.0 / (2.5 * lag))
    for tag in ("ROLL", "PITCH", "YAW"):
        p = t[f"MC_{tag}RATE_P"]
        assert 0.02 <= p <= 0.6
        if tag != "YAW":
            assert t[f"MC_{tag}_P"] == pytest.approx(w_c / 2.5, abs=1e-3)
        assert t[f"MC_{tag}RATE_K"] == 1.0
    # outer loops sized below the attitude bandwidth (the quad defaults drove a 0.25 Hz limit cycle)
    w_att = w_c / 2.5
    assert (
        t["MPC_XY_VEL_P_ACC"] == pytest.approx(0.75 * w_att, abs=1e-3)
        and t["MPC_TILTMAX_AIR"] == 20.0
    )
    assert (
        t["MPC_XY_P"] < 0.95
        and t["MPC_Z_VEL_P_ACC"] < 4.0
        and t["MC_YAW_P"] == pytest.approx(1.4 * w_att, abs=1e-3)
    )
    # pitch has the least authority per inertia, so it needs the largest rate gain
    assert t["MC_PITCHRATE_P"] > t["MC_ROLLRATE_P"]
    assert t["MC_YAWRATE_D"] == 0.0
    assert t["MC_PITCHRATE_D"] == pytest.approx(0.02 * t["MC_PITCHRATE_P"], abs=1e-5)


def test_unattainable_axes_keep_defaults(cad):
    # the all-45-degree set cannot trim, so only the thrust model factor is written
    t = px4_tuning(cad)
    assert t["THR_MDL_FAC"] == 1.0 and not any(k.startswith("MC_") for k in t)
    assert "MPC_TILTMAX_AIR" in t  # outer-loop limits are geometry independent


def test_export_carries_tuning_lag_and_mass(hover, tmp_path):
    out = export_gazebo(hover, tmp_path)
    airframe = open(out["airframe"], encoding="utf-8").read()
    assert f"{airframe_id(hover.meta.name)}_gz_" in out["airframe"]
    assert "param set-default THR_MDL_FAC 1" in airframe
    assert "param set-default SIM_GZ_EC_MIN1 0" in airframe  # zero idle: thrust linear in command
    assert (
        "param set-default MC_PITCHRATE_P" in airframe
        and "param set-default MPC_THR_HOVER" in airframe
    )
    model = ET.parse(out["model_sdf"]).getroot().find("model")
    base = next(link for link in model.findall("link") if link.get("name") == "base_link")
    assert float(base.find("inertial/mass").text) == pytest.approx(body_mass_kg(hover))
    rotor_total = sum(
        float(link.find("inertial/mass").text)
        for link in model.findall("link")
        if link.get("name").startswith("rotor_")
    )
    assert body_mass_kg(hover) + rotor_total == pytest.approx(hover.mass.total_kg)
    assert rotor_total == pytest.approx(ROTOR_MASS_KG * len(hover.fans))
    plugin = next(p for p in model.findall("plugin") if "MulticopterMotorModel" in p.get("name"))
    assert float(plugin.find("timeConstantUp").text) == pytest.approx(0.15)
    readme = open(out["readme"], encoding="utf-8").read()
    assert "## PX4 controller sizing" in readme and "MC_PITCHRATE_P" in readme
