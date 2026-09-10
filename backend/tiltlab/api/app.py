"""FastAPI application entry point.

Serves the built frontend from ``frontend/dist`` when it exists and exposes the
JSON API under ``/api``. Run with ``uvicorn tiltlab.api.app:app``.
"""

from __future__ import annotations

from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import FileResponse, HTMLResponse, Response
from fastapi.staticfiles import StaticFiles

from tiltlab import __version__

REPO_ROOT = Path(__file__).resolve().parents[3]
FRONTEND_DIST = REPO_ROOT / "frontend" / "dist"
FRONTEND_INDEX = FRONTEND_DIST / "index.html"
FRONTEND_ASSETS = FRONTEND_DIST / "assets"

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


@app.get("/api/health")
def health() -> dict[str, str]:
    """Liveness probe used by the frontend and docker healthcheck."""
    return {"status": "ok", "version": __version__}


@app.get("/", include_in_schema=False)
def index() -> Response:
    """Serve the built SPA, or a minimal placeholder when no build exists."""
    if FRONTEND_INDEX.is_file():
        return FileResponse(FRONTEND_INDEX, media_type="text/html")
    return HTMLResponse(PLACEHOLDER_HTML)


if FRONTEND_ASSETS.is_dir():
    app.mount("/assets", StaticFiles(directory=FRONTEND_ASSETS), name="assets")
