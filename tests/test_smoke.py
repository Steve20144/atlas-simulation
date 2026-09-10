"""Scaffold smoke test: package imports and the API serves health and index."""

from fastapi.testclient import TestClient

import tiltlab
from tiltlab.api.app import app


def test_package_has_version() -> None:
    assert tiltlab.__version__


def test_health() -> None:
    with TestClient(app) as client:
        response = client.get("/api/health")
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "ok"
    assert body["version"] == tiltlab.__version__


def test_index_serves_html() -> None:
    with TestClient(app) as client:
        response = client.get("/")
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("text/html")
    assert "tiltlab" in response.text.lower()
