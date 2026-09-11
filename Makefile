SHELL := bash
.ONESHELL:
.PHONY: menu dev test lint docker guide

UV      := uv run --project backend
NPM     := npm --prefix frontend

# Interactive menu over the whole stack (app, SITL, HITL, exports, firmware, logs).
menu:
	$(UV) python scripts/tiltlab_menu.py

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

# Gazebo and PX4 guide, docs/guide/gazebo_px4_guide.pdf (needs a LaTeX distribution with latexmk;
# MiKTeX on this machine). Byproducts stay in docs/guide/build.
guide:
	cd docs/guide && latexmk -pdf -interaction=nonstopmode -outdir=build gazebo_px4_guide.tex
	cp docs/guide/build/gazebo_px4_guide.pdf docs/guide/gazebo_px4_guide.pdf
