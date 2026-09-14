"""PX4 v1.17.0 parameter catalogue: names, descriptions, types, ranges and enum labels from the
pinned tree, built by scripts/build_param_catalog.py into param_catalog.json next to this file.
The catalogue covers the modules in the sparse checkout plus a curated supplement; the board,
not the catalogue, decides whether a name exists."""

from __future__ import annotations

import json
import re
from functools import lru_cache
from pathlib import Path
from typing import Any

CATALOG_PATH = Path(__file__).with_name("param_catalog.json")
NAME_RE = re.compile(r"^[A-Z][A-Z0-9_]{1,15}$")


@lru_cache(maxsize=1)
def _load() -> dict[str, dict[str, Any]]:
    data = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
    return {p["name"]: p for p in data["params"]}


def count() -> int:
    return len(_load())


def lookup(name: str) -> dict[str, Any] | None:
    return _load().get(name.strip().upper())


def search(query: str, limit: int = 30) -> list[dict[str, Any]]:
    """Case-insensitive; every whitespace-separated token must occur in the name, the short or
    the long description. Ranked by where the first token hits: name prefix, name, short, long;
    ties alphabetical. An empty query lists the first `limit` names."""
    toks = [t for t in query.lower().split() if t]
    cat = _load()
    if not toks:
        return [cat[k] for k in sorted(cat)[:limit]]
    scored: list[tuple[int, str, dict[str, Any]]] = []
    for name, p in cat.items():
        n, s, lg = name.lower(), p["short"].lower(), p["long"].lower()
        if not all(t in n or t in s or t in lg for t in toks):
            continue
        t0 = toks[0]
        rank = 0 if n.startswith(t0) else 1 if t0 in n else 2 if t0 in s else 3
        scored.append((rank, name, p))
    scored.sort(key=lambda x: (x[0], x[1]))
    return [p for _, _, p in scored[:limit]]
