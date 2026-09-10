"""Sweep wing-fan tilt angles for a scenario and rank geometries by hover power.

    uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json \
        [--tilts 0:45:5 | --tilts 0,10,20,30] [--mode inward|outward|forward|aft] [--per-pair] \
        [--centre 0,10] [--concept stock] [--collective 0.35] [--min-headroom 0.2] [--min-yaw 0.5] \
        [--top 15] [--csv exports]

Writes a timestamped CSV of every candidate when --csv is given and prints the ranked table.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tiltlab.core.sweep import SweepSpec, run_sweep, sweep_table
from tiltlab.export.csv_export import export_metrics_csv
from tiltlab.scenario import Scenario


def parse_angles(text: str) -> list[float]:
    if ":" in text:
        start, stop, step = (float(x) for x in text.split(":"))
        out, t = [], start
        while t <= stop + 1e-9:
            out.append(round(t, 6))
            t += step
        return out
    return [float(x) for x in text.split(",") if x.strip()]


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scenario")
    ap.add_argument("--tilts", default="0:45:5")
    ap.add_argument("--mode", default="inward", choices=["inward", "outward", "forward", "aft"])
    ap.add_argument("--per-pair", action="store_true")
    ap.add_argument("--centre", default="0")
    ap.add_argument("--concept", default="stock", choices=["stock", "fully_actuated"])
    ap.add_argument("--collective", type=float, default=None)
    ap.add_argument("--min-headroom", type=float, default=0.2)
    ap.add_argument("--min-yaw", type=float, default=0.0)
    ap.add_argument("--top", type=int, default=15)
    ap.add_argument("--csv", default=None, help="directory for the timestamped CSV")
    args = ap.parse_args()

    scenario = Scenario.model_validate(json.loads(Path(args.scenario).read_text(encoding="utf-8")))
    spec = SweepSpec(
        tilts_deg=parse_angles(args.tilts),
        azimuth_mode=args.mode,
        per_pair=args.per_pair,
        centreline_tilts_deg=parse_angles(args.centre),
        concept=args.concept,
        collective=args.collective,
        min_headroom=args.min_headroom,
        min_yaw_Nm=args.min_yaw,
    )
    result = run_sweep(scenario, spec)
    print(
        f"{scenario.meta.name}: {result['n_evaluated']} candidates, "
        f"{result['n_feasible']} feasible, {result['elapsed_ms']:.0f} ms, "
        f"concept {spec.concept}, azimuth {spec.azimuth_mode}"
        + (" (truncated)" if result["truncated"] else "")
    )
    if scenario.mass.estimated or any(c.estimated for c in scenario.fan_curves.values()):
        print("inputs are ESTIMATED (mass or fan curve): compare candidates relatively")
    print(sweep_table(result, args.top))
    best = result["best"]
    if best:
        print(
            f"\nbest: pair tilts {best['pair_tilts_deg']} deg, {best['power_W']:.0f} W, "
            f"yaw {best['yaw_Nm']:.2f} N m"
        )
    if args.csv:
        rows = [{"rank": i + 1, **c} for i, c in enumerate(result["candidates"])]
        path = export_metrics_csv(
            rows, args.csv, stem=f"{scenario.meta.name}_sweep_{spec.azimuth_mode}"
        )
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
