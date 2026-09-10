"""CSV metrics export and Scenario JSON export (PLAN.md M9)."""

from __future__ import annotations

import csv
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

from tiltlab.export.naming import timestamped_name
from tiltlab.scenario import Scenario


def flatten_row(row: Mapping[str, Any], prefix: str = "", sep: str = ".") -> dict[str, Any]:
    """Flatten nested dicts ('a.b') and lists/tuples ('a.0') into scalar columns."""
    out: dict[str, Any] = {}
    for key, value in row.items():
        name = f"{prefix}{sep}{key}" if prefix else str(key)
        if isinstance(value, Mapping):
            out.update(flatten_row(value, name, sep))
        elif isinstance(value, Sequence) and not isinstance(value, str | bytes):
            out.update(flatten_row({str(i): v for i, v in enumerate(value)}, name, sep))
        else:
            out[name] = value
    return out


def csv_columns(rows: Sequence[Mapping[str, Any]]) -> list[str]:
    """Union of the flattened keys in first-seen order."""
    cols: dict[str, None] = {}
    for r in rows:
        cols.update(dict.fromkeys(flatten_row(r)))
    return list(cols)


def export_metrics_csv(
    rows: Sequence[Mapping[str, Any]],
    out_dir: str | Path,
    stem: str = "metrics",
    now: datetime | None = None,
) -> Path:
    """One CSV row per scenario/concept entry of `rows` (each a dict, possibly nested; keys such
    as 'scenario' and 'concept' become plain columns). Returns the written path."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / timestamped_name(stem, "csv", now)
    columns = csv_columns(rows)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for r in rows:
            writer.writerow(flatten_row(r))
    return path


def export_scenario_json(
    scenario: Scenario, out_dir: str | Path, now: datetime | None = None
) -> Path:
    """Write the Scenario as '<stamp>_<name>.json' (indent 2) and return the path."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / timestamped_name(scenario.meta.name, "json", now)
    path.write_text(scenario.model_dump_json(indent=2) + "\n", encoding="utf-8")
    return path
