"""Gazebo Classic HITL harness: model structure, HIL serial interface, parameter file."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from fastapi.testclient import TestClient

from tiltlab.api.app import app
from tiltlab.core.params_px4 import read_params_file
from tiltlab.export.gazebo import frd_to_flu
from tiltlab.export.gazebo_classic_hitl import export_gazebo_classic_hitl, hitl_params
from tiltlab.export.params import ca_geometry_params
from tiltlab.scenario import Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


@pytest.fixture(scope="module")
def cad() -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))


def test_hitl_params(cad):
    p = hitl_params(cad)
    assert p["SYS_AUTOSTART"] == 1001 and p["SYS_HITL"] == 1 and p["CA_ROTOR_COUNT"] == 10
    assert [p[f"HIL_ACT_FUNC{i}"] for i in range(1, 11)] == list(range(101, 111))
    ca = ca_geometry_params(cad)
    assert p["CA_ROTOR0_AX"] == ca["CA_ROTOR0_AX"]


def test_export(cad, tmp_path):
    out = export_gazebo_classic_hitl(cad, tmp_path)
    root = ET.parse(out["model_sdf"]).getroot()
    assert root.get("version") == "1.6"
    model = root.find("model")
    links = [link.get("name") for link in model.findall("link")]
    assert "base_link" in links and "/imu_link" in links
    assert sum(name.startswith("rotor_") for name in links) == 10
    plugins = {p.get("name"): p for p in model.findall("plugin")}
    assert sum(1 for n in plugins if n.startswith("motor_")) == 10
    mav = plugins["mavlink_interface"]
    assert mav.find("serialEnabled").text == "1" and mav.find("hil_mode").text == "1"
    assert mav.find("baudRate").text == "921600" and mav.find("use_tcp").text == "0"
    channels = mav.find("control_channels").findall("channel")
    assert len(channels) == 10 and channels[9].find("input_index").text == "9"
    for name in (
        "rotors_gazebo_imu_plugin",
        "magnetometer_plugin",
        "barometer_plugin",
        "groundtruth_plugin",
    ):
        assert name in plugins
    assert model.find("model").get("name") == "gps0"
    ca = ca_geometry_params(cad)
    mc = float(plugins["motor_0_model"].find("motorConstant").text)
    assert mc * 1000.0**2 == pytest.approx(ca["CA_ROTOR0_CT"], rel=1e-6)
    rotor0 = next(link for link in model.findall("link") if link.get("name") == "rotor_0")
    pose = [float(v) for v in rotor0.find("pose").text.split()]
    expected = frd_to_flu(cad.effective_pos(cad.fans_sorted()[0]) - np.asarray(cad.mass.cg_frd_m))
    assert pose[:3] == pytest.approx(expected, abs=1e-4)
    ET.parse(out["world"])
    pf = read_params_file(out["params"])
    d = pf.to_dict()
    assert d["SYS_HITL"] == 1 and d["HIL_ACT_FUNC10"] == 110 and d["CA_ROTOR_COUNT"] == 10
    assert "pwm_out_sim" in open(out["readme"], encoding="utf-8").read()
    if out["mesh"]:
        assert out["mesh"].endswith("airframe.stl")


def test_endpoint(cad):
    client = TestClient(app)
    r = client.post("/api/export/gazebo_hitl", json={"scenario": cad.model_dump(mode="json")})
    assert r.status_code == 200, r.text
    assert r.json()["params"].endswith(".params")
