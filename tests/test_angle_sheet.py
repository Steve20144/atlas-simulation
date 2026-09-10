"""Foil design sheet: CAD-frame directions and coordinates for each motor's deflection."""

from __future__ import annotations

import json

import numpy as np
import pytest
from fastapi.testclient import TestClient

from tiltlab.api.app import app
from tiltlab.cad.fusion_dashboard import FusionFrame
from tiltlab.core.sweep import apply_deflections
from tiltlab.export.angle_sheet import cad_rotation, export_angle_sheet, foil_angle_sheet
from tiltlab.scenario import Scenario

from .conftest import FIXTURES

SCENARIOS = FIXTURES.parent.parent / "scenarios"


@pytest.fixture(scope="module")
def cad() -> Scenario:
    return Scenario.model_validate(json.loads((SCENARIOS / "atlas_phase01_cad.json").read_text()))


def test_cad_rotation_matches_import_frame(cad):
    assert np.allclose(cad_rotation(cad), FusionFrame(forward="+z", up="-y").rotation())
    assert cad.frame.cad_origin is not None


def test_sheet_rows_as_built(cad):
    rows = foil_angle_sheet(cad)
    assert [r["rotor"] for r in rows] == list(range(8))
    r0 = rows[0]
    assert r0["deflection_deg"] == 45.0 and r0["change_from_as_built_deg"] == 0.0
    assert r0["exhaust_angle_below_fore_aft_deg"] == pytest.approx(45.0)
    # exhaust aft and down in FRD; in Fusion (forward +z, up -y) that is -z and +y
    assert r0["exhaust_dir_frd"] == "(-0.707, 0.000, 0.707)"
    assert r0["exhaust_dir_cad"] == "(0.000, 0.707, -0.707)"
    assert r0["thrust_dir_cad"] == "(0.000, -0.707, 0.707)"
    # CAD coordinates: the motor centre must come back to the dashboard's centroid of that EDF
    motor_cad = np.asarray(r0["motor_centre_cad_mm"])
    assert np.allclose(motor_cad, [371.0, 276.5, -159.7], atol=0.6) or np.allclose(
        motor_cad, [-371.0, 276.5, -159.7], atol=0.6
    )
    pp = np.asarray(r0["pressure_point_cad_mm"])
    assert pp[2] < motor_cad[2]  # pressure point aft (Fusion -z) of the motor


def test_sheet_follows_deflection_changes(cad):
    sc = apply_deflections(cad, {0: 90.0, 1: 90.0, 6: 135.0, 7: 135.0})
    rows = {r["rotor"]: r for r in foil_angle_sheet(sc)}
    assert rows[0]["change_from_as_built_deg"] == 45.0
    assert rows[0]["exhaust_dir_cad"] == "(0.000, 1.000, 0.000)"  # straight down in Fusion (+y)
    assert rows[6]["exhaust_angle_below_fore_aft_deg"] == pytest.approx(135.0)
    assert rows[6]["thrust_dir_frd"] == "(-0.707, 0.000, -0.707)"  # up and aft
    assert rows[2]["deflection_deg"] == 45.0


def test_export_and_endpoints(cad, tmp_path):
    csv_path, md_path = export_angle_sheet(cad, tmp_path)
    assert csv_path.exists() and md_path.exists()
    text = md_path.read_text(encoding="utf-8")
    assert "Foil design sheet" in text and "| 0 | left" in text or "| 0 | right" in text
    client = TestClient(app)
    r = client.post("/api/foil_sheet", json={"scenario": cad.model_dump(mode="json")})
    assert r.status_code == 200, r.text
    assert len(r.json()["rows"]) == 8 and "exhaust_dir_cad" in r.json()["rows"][0]
