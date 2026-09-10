SHELL := bash
.ONESHELL:
.PHONY: dev test lint docker

UV      := uv run --project backend
NPM     := npm --prefix frontend

# Backend on :8000 (uvicorn --reload) and Vite dev server on :5173, together.
dev:
	set -euo pipefail
	$(UV) uvicorn tiltlab.api.app:app --reload --port 8000 &
	backend_pid=$$!
	trap 'kill $$backend_pid 2>/dev/null || true' EXIT INT TERM
	$(NPM) run dev
	wait $$backend_pid

test:
	set -euo pipefail
	$(UV) pytest tests -q
	$(NPM) run test

lint:
	set -euo pipefail
	$(UV) ruff check backend tests
	$(NPM) run lint

docker:
	docker compose build
