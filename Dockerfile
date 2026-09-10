# tiltlab analysis server. M10 finalises this image; M1 keeps it minimal.
# Build context is the repo root; frontend/dist must exist (npm --prefix frontend run build).
FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    UV_LINK_MODE=copy \
    UV_PROJECT_ENVIRONMENT=/app/backend/.venv \
    PATH="/app/backend/.venv/bin:${PATH}"

# OpenCascade (cadquery-ocp) and matplotlib need a few shared libraries.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libgl1 libglu1-mesa libxrender1 libxext1 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app

COPY backend/ backend/
RUN uv sync --project backend --frozen --no-dev

COPY frontend/dist/ frontend/dist/
# scenarios/ is bind-mounted by docker-compose; create the mount point in the image.
RUN mkdir -p scenarios

EXPOSE 8000
CMD ["uvicorn", "tiltlab.api.app:app", "--host", "0.0.0.0", "--port", "8000"]
