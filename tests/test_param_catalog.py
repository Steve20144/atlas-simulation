"""PX4 parameter catalogue (backend/vectra/px4/param_catalog.json) and its endpoints."""

from __future__ import annotations

from fastapi.testclient import TestClient

from vectra.api.app import app
from vectra.px4 import param_catalog


def test_catalog_entries_from_c_yaml_and_supplement() -> None:
    assert param_catalog.count() > 500
    air = param_catalog.lookup("mc_airmode")
    assert air is not None and air["type"] == "enum" and air["group"] == "Mixer Output"
    assert air["values"] == {"0": "Disabled", "1": "Roll/Pitch", "2": "Roll/Pitch/Yaw"}
    assert "diverge" in air["long"]
    rotor = param_catalog.lookup("CA_ROTOR9_PY")
    assert rotor is not None and "rotor 9" in rotor["short"] and rotor["unit"] == "m"
    y_off = param_catalog.lookup("SENS_BOARD_Y_OFF")
    assert y_off is not None and (y_off["unit"], y_off["min"], y_off["max"]) == ("deg", -45, 45)
    hover = param_catalog.lookup("MPC_THR_HOVER")
    assert hover is not None and "Copyright" not in hover["short"] and hover["max"] == 0.8
    hitl = param_catalog.lookup("SYS_HITL")
    assert hitl is not None and hitl["reboot"] is True and hitl["values"]["1"].startswith("HITL")
    assert param_catalog.lookup("NOPE_X") is None


def test_search_ranking() -> None:
    assert [p["name"] for p in param_catalog.search("airmode")][0] == "MC_AIRMODE"
    names = [p["name"] for p in param_catalog.search("rotor 3 position y")]
    assert "CA_ROTOR3_PY" in names[:3]
    assert len(param_catalog.search("", limit=5)) == 5
    assert param_catalog.search("zzzznotaparam") == []
    first = [p["name"] for p in param_catalog.search("ca_rotor0", limit=3)][0]
    assert first.startswith("CA_ROTOR0")


def test_endpoints() -> None:
    with TestClient(app) as c:
        r = c.get("/api/px4/params?q=air-mode&limit=3").json()
        assert r["count"] > 500 and r["results"][0]["name"] == "MC_AIRMODE"
        assert len(c.get("/api/px4/params?limit=2").json()["results"]) == 2
        assert c.get("/api/px4/params?limit=0").status_code == 422
        assert c.get("/api/px4/params/MC_AIRMODE").json()["values"]["0"] == "Disabled"
        assert c.get("/api/px4/params/NOPE_X").status_code == 404
