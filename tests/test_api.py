"""REST API round trips for every endpoint and the /api/metrics latency budget (< 50 ms warm)."""

from __future__ import annotations

import json
import shutil
import time
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

import tiltlab.api.app as app_module
from tiltlab.api.app import app

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"
LATENCY_BUDGET_MS = 50.0


@pytest.fixture()
def client(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> TestClient:
    """TestClient over a temp copy of scenarios/, so saves never touch the repo."""
    scen_dir = tmp_path / "scenarios"
    shutil.copytree(SCENARIOS, scen_dir)
    monkeypatch.setattr(app_module, "SCENARIOS_DIR", scen_dir)
    with TestClient(app) as c:
        yield c


def scenario_json(name: str) -> dict:
    return json.loads((SCENARIOS / f"{name}.json").read_text())


def test_health_and_index(client: TestClient) -> None:
    assert client.get("/api/health").json()["status"] == "ok"
    assert client.get("/").status_code == 200


def test_cors_allows_vite_origin(client: TestClient) -> None:
    r = client.options(
        "/api/metrics",
        headers={"Origin": "http://localhost:5173", "Access-Control-Request-Method": "POST"},
    )
    assert r.status_code == 200
    assert r.headers["access-control-allow-origin"] == "http://localhost:5173"


def test_list_get_save_scenarios(client: TestClient) -> None:
    names = client.get("/api/scenarios").json()["scenarios"]
    assert "baseline_dihedral30" in names and "flown_log40_vertical_km" in names

    r = client.get("/api/scenarios/baseline_dihedral30")
    assert r.status_code == 200
    body = r.json()
    assert body["meta"]["name"] == "baseline_dihedral30" and len(body["fans"]) == 10

    assert client.get("/api/scenarios/does_not_exist").status_code == 404
    assert client.get("/api/scenarios/..%2Fetc").status_code in (400, 404)

    body["fans"][0]["tilt_deg"] = 12.5
    r = client.post("/api/scenarios/copy_test", json=body)
    assert r.status_code == 200, r.text
    saved = Path(r.json()["path"])
    assert saved.is_file() and saved.parent == app_module.SCENARIOS_DIR
    assert "copy_test" in client.get("/api/scenarios").json()["scenarios"]
    assert client.get("/api/scenarios/copy_test").json()["fans"][0]["tilt_deg"] == 12.5

    body["fans"] = body["fans"][:9]
    assert client.post("/api/scenarios/bad", json=body).status_code == 422


def test_px4_params_preview(client: TestClient) -> None:
    sc = scenario_json("baseline_dihedral30")
    r = client.post("/api/px4_params_preview", json={"scenario": sc, "concept": "stock"})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["params"]["CA_ROTOR_COUNT"] == 10
    assert body["params"]["CA_METHOD"] == 2
    assert body["extras"] == {}
    assert any(line.startswith("CA_ROTOR0_PX\t") for line in body["lines"])
    assert body["params"]["CA_ROTOR0_CT"] == pytest.approx(33.3, abs=1e-4)

    r = client.post("/api/px4_params_preview", json={"scenario": sc, "concept": "fully_actuated"})
    body = r.json()
    assert body["params"]["CA_METHOD"] == 0
    assert body["extras"] == {"FD_FAIL_P": 0, "FD_FAIL_R": 0}
    assert "FD_FAIL_P\t0" in body["lines"] and "CA_METHOD\t0" in body["lines"]


def test_metrics_round_trip_and_latency(client: TestClient) -> None:
    sc = scenario_json("flown_log40_vertical_km")
    req = {"scenario": sc, "concept": "stock", "blend": 0.0, "collective": None, "weights": None}
    r = client.post("/api/metrics", json=req)
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["concept"] == "stock" and body["controlled_axes"] == ["roll", "pitch", "yaw", "Fz"]
    assert body["hover"]["exact"] is True
    assert set(body["authority"]) == {"roll", "pitch", "yaw", "Fz"}
    assert body["estimated"] is True

    # concept defaults to the scenario's, weights are validated
    assert client.post("/api/metrics", json={"scenario": sc}).json()["concept"] == "stock"
    assert (
        client.post("/api/metrics", json={"scenario": sc, "weights": {"x": 1}}).status_code == 400
    )
    fa = client.post("/api/metrics", json={"scenario": sc, "concept": "fully_actuated"}).json()
    assert len(fa["controlled_axes"]) == 6

    # warm latency, wall clock around the full HTTP round trip
    timings = []
    for _ in range(5):
        t0 = time.perf_counter()
        assert client.post("/api/metrics", json=req).status_code == 200
        timings.append((time.perf_counter() - t0) * 1e3)
    best = min(timings)
    median = sorted(timings)[len(timings) // 2]
    print(f"\n /api/metrics warm latency: best {best:.1f} ms, median {median:.1f} ms")
    assert median < LATENCY_BUDGET_MS, f"median {median:.1f} ms exceeds {LATENCY_BUDGET_MS} ms"


def test_export_endpoints_write_into_exports_dir(
    client: TestClient, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    out = tmp_path / "exports"
    monkeypatch.setattr(app_module, "EXPORTS_DIR", out)
    scenario = scenario_json("baseline_dihedral30")

    r = client.post(
        "/api/export/params", json={"scenario": scenario, "concept": "fully_actuated"}
    )
    assert r.status_code == 200, r.text
    path = Path(r.json()["path"])
    assert path.parent == out and path.suffix == ".params"
    assert "baseline_dihedral30_fully_actuated_px4" in path.name
    assert "CA_METHOD\t0" in path.read_text(encoding="utf-8")

    r = client.post(
        "/api/export/params", json={"scenario": scenario, "base": str(tmp_path / "missing.params")}
    )
    assert r.status_code == 400

    rows = [
        {"scenario": "a", "hover": {"power_W": 1.5}},
        {"scenario": "b", "hover": {"power_W": 2.0}},
    ]
    r = client.post("/api/export/csv", json={"rows": rows, "stem": "snapshot"})
    assert r.status_code == 200, r.text
    csv_path = Path(r.json()["path"])
    lines = csv_path.read_text(encoding="utf-8").splitlines()
    assert csv_path.parent == out and lines[0] == "scenario,hover.power_W" and len(lines) == 3
    assert client.post("/api/export/csv", json={"rows": rows, "stem": "../x"}).status_code == 400
