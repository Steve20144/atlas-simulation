"""FastAPI application entry point.

Serves the built frontend from ``frontend/dist`` when it exists and exposes the
JSON API under ``/api``. Run with ``uvicorn tiltlab.api.app:app``.
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse, Response
from fastapi.staticfiles import StaticFiles

from tiltlab import __version__
from tiltlab.api.schemas import (
    ExportCsvRequest,
    ExportParamsRequest,
    ExportResponse,
    MetricsRequest,
    MetricsResponse,
    Px4ParamsPreviewRequest,
    Px4ParamsPreviewResponse,
    ScenarioListResponse,
    ScenarioSaveResponse,
)
from tiltlab.core.metrics import compute_metrics
from tiltlab.core.params_px4 import (
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_INT32,
    format_param_value,
    scenario_to_ca_params,
)
from tiltlab.export import export_metrics_csv, export_params
from tiltlab.scenario import Scenario

REPO_ROOT = Path(__file__).resolve().parents[3]
FRONTEND_DIST = REPO_ROOT / "frontend" / "dist"
FRONTEND_INDEX = FRONTEND_DIST / "index.html"
FRONTEND_ASSETS = FRONTEND_DIST / "assets"
SCENARIOS_DIR = Path(os.environ.get("TILTLAB_SCENARIOS_DIR", REPO_ROOT / "scenarios"))
EXPORTS_DIR = Path(os.environ.get("TILTLAB_EXPORTS_DIR", REPO_ROOT / "exports"))
SCENARIO_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")
CORS_ORIGINS = ["http://localhost:5173", "http://127.0.0.1:5173"]

INT_PARAMS = {"CA_METHOD", "CA_ROTOR_COUNT", "CA_R_REV", "CA_AIRFRAME", "FD_FAIL_P", "FD_FAIL_R"}
# Fully actuated concept: pseudo-inverse allocation (no yaw-first desaturation) and the
# attitude failure detector disabled, because the tilted trim attitude would trip it.
FULLY_ACTUATED_EXTRAS: dict[str, int | float] = {"CA_METHOD": 0, "FD_FAIL_P": 0, "FD_FAIL_R": 0}

PLACEHOLDER_HTML = """<!doctype html>
<html lang="en">
<head><meta charset="utf-8"><title>tiltlab</title></head>
<body>
<h1>tiltlab</h1>
<p>Frontend build not found. Run <code>npm --prefix frontend run build</code>
or use <code>make dev</code> for the Vite dev server on port 5173.</p>
</body>
</html>
"""

app = FastAPI(title="tiltlab", version=__version__)
app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict[str, str]:
    """Liveness probe used by the frontend and docker healthcheck."""
    return {"status": "ok", "version": __version__}


@app.post("/api/metrics", response_model=MetricsResponse)
def metrics(req: MetricsRequest) -> MetricsResponse:
    """Static metrics (core/metrics.py) for a scenario under a concept at a collective."""
    concept = req.concept or req.scenario.control.concept
    blend = req.scenario.control.blend if req.blend is None else req.blend
    try:
        result = compute_metrics(
            req.scenario, concept, collective=req.collective, weights=req.weights
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    result["blend"] = float(blend)
    return MetricsResponse.model_validate(result)


def _param_type(name: str) -> int:
    return PARAM_TYPE_INT32 if name in INT_PARAMS else PARAM_TYPE_FLOAT


def px4_params_preview(scenario: Scenario, concept: str) -> Px4ParamsPreviewResponse:
    """CA_* parameters of the scenario plus the concept extras, and one text line per entry."""
    params = scenario_to_ca_params(scenario)
    extras: dict[str, int | float] = (
        dict(FULLY_ACTUATED_EXTRAS) if concept == "fully_actuated" else {}
    )
    if "CA_METHOD" in extras:
        params["CA_METHOD"] = extras.pop("CA_METHOD")
    merged = {**params, **extras}
    lines = [f"{name}\t{format_param_value(v, _param_type(name))}" for name, v in merged.items()]
    return Px4ParamsPreviewResponse(concept=concept, params=params, extras=extras, lines=lines)


@app.post("/api/px4_params_preview", response_model=Px4ParamsPreviewResponse)
def px4_params_preview_endpoint(req: Px4ParamsPreviewRequest) -> Px4ParamsPreviewResponse:
    return px4_params_preview(req.scenario, req.concept or req.scenario.control.concept)


def _scenario_path(name: str) -> Path:
    if not SCENARIO_NAME_RE.match(name) or ".." in name:
        raise HTTPException(status_code=400, detail="invalid scenario name")
    return SCENARIOS_DIR / f"{name}.json"


@app.get("/api/scenarios", response_model=ScenarioListResponse)
def list_scenarios() -> ScenarioListResponse:
    """Names (without .json) of the scenario files in the scenarios directory."""
    if not SCENARIOS_DIR.is_dir():
        return ScenarioListResponse(scenarios=[])
    return ScenarioListResponse(scenarios=sorted(p.stem for p in SCENARIOS_DIR.glob("*.json")))


@app.get("/api/scenarios/{name}", response_model=Scenario)
def get_scenario(name: str) -> Scenario:
    path = _scenario_path(name)
    if not path.is_file():
        raise HTTPException(status_code=404, detail=f"scenario '{name}' not found")
    try:
        return Scenario.model_validate(json.loads(path.read_text(encoding="utf-8")))
    except ValueError as exc:
        raise HTTPException(status_code=500, detail=f"scenario file invalid: {exc}") from exc


@app.post("/api/scenarios/{name}", response_model=ScenarioSaveResponse)
def save_scenario(name: str, scenario: Scenario) -> ScenarioSaveResponse:
    """Validate and write the scenario to scenarios/<name>.json (creates the directory)."""
    path = _scenario_path(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(scenario.model_dump_json(indent=2) + "\n", encoding="utf-8")
    return ScenarioSaveResponse(name=name, path=str(path))


@app.post("/api/export/params", response_model=ExportResponse)
def export_params_endpoint(req: ExportParamsRequest) -> ExportResponse:
    """Write a timestamped PX4 .params file (tiltlab.export.export_params) into exports/."""
    concept = req.concept or req.scenario.control.concept
    if req.base is not None and not Path(req.base).is_file():
        raise HTTPException(status_code=400, detail=f"base params file not found: {req.base}")
    try:
        path = export_params(req.scenario, concept, req.base, out_dir=EXPORTS_DIR)
    except (ValueError, OSError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return ExportResponse(path=str(path))


@app.post("/api/export/csv", response_model=ExportResponse)
def export_csv_endpoint(req: ExportCsvRequest) -> ExportResponse:
    """Write the rows (nested dicts flattened to a.b columns) as a timestamped CSV in exports/."""
    if not SCENARIO_NAME_RE.match(req.stem):
        raise HTTPException(status_code=400, detail="invalid stem")
    path = export_metrics_csv(req.rows, EXPORTS_DIR, stem=req.stem)
    return ExportResponse(path=str(path))


@app.get("/", include_in_schema=False)
def index() -> Response:
    """Serve the built SPA, or a minimal placeholder when no build exists."""
    if FRONTEND_INDEX.is_file():
        return FileResponse(FRONTEND_INDEX, media_type="text/html")
    return HTMLResponse(PLACEHOLDER_HTML)


if FRONTEND_ASSETS.is_dir():
    app.mount("/assets", StaticFiles(directory=FRONTEND_ASSETS), name="assets")


# ---------------------------------------------------------------- tilt sweep
from tiltlab.api.sweep_schemas import SweepRequest, SweepResponse  # noqa: E402
from tiltlab.core.sweep import SweepSpec, run_sweep  # noqa: E402


@app.post("/api/sweep", response_model=SweepResponse)
def sweep_endpoint(req: SweepRequest) -> SweepResponse:
    """Grid sweep over wing-fan tilt; candidates ranked by hover power among those that keep
    control."""
    try:
        spec = SweepSpec(
            tilts_deg=req.tilts_deg,
            variable=req.variable,
            foil_grouping=req.foil_grouping,
            azimuth_mode=req.azimuth_mode,
            per_pair=req.per_pair,
            centreline_tilts_deg=req.centreline_tilts_deg,
            centreline_azimuth_deg=req.centreline_azimuth_deg,
            concept=req.concept,
            collective=req.collective,
            min_headroom=req.min_headroom,
            min_yaw_Nm=req.min_yaw_Nm,
            rank_by=req.rank_by,
        )
        result = run_sweep(req.scenario, spec)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    result["candidates"] = result["candidates"][: req.top]
    return SweepResponse(**result)


@app.get("/api/cad/model/{name}", include_in_schema=False)
def cad_model(name: str) -> Response:
    """glTF binary of the airframe meshes written next to the scenario by the CAD import."""
    path = _scenario_path(name).with_suffix(".glb")
    if not path.exists():
        raise HTTPException(status_code=404, detail="no CAD model for this scenario")
    return FileResponse(path, media_type="model/gltf-binary")
