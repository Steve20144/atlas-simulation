"""Build a tiltlab scenario from the Atlas Mass & CoG dashboard HTML.

    uv run --project backend python scripts/import_cad_dashboard.py
    tests/fixtures/cog_dashboard_full_8.html \
        [--weights atlas_weights.json] [--template scenarios/flown_log40_vertical_km.json] \
        [--params tests/fixtures/20260909_1515_params_v4_rig_airmode.params] [--up +y] [--out
        scenarios/atlas_phase01_cad.json]

Without --weights the fan positions are measured from the reference CG fitted to the CA_ROTOR
positions of --params
and the mass stays the template placeholder (flagged estimated). Export the weights from the
dashboard (Export
button, JSON with weights_by_type) and pass them with --weights to get the real mass, CG and
inertia.
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np

from tiltlab.cad.fusion_dashboard import (
    FusionFrame,
    Weights,
    build_scenario,
    detect_edfs,
    fit_reference_cg,
    load_dashboard,
    order_edfs_px4,
)
from tiltlab.core.params_px4 import read_params_file
from tiltlab.scenario import Scenario

ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("html")
    ap.add_argument("--weights", help="dashboard export JSON (weights_by_type in grams)")
    ap.add_argument("--template", default=str(ROOT / "scenarios" / "baseline_dihedral30.json"))
    ap.add_argument(
        "--no-foils", action="store_true", help="raw fan tilt instead of the foil model"
    )
    ap.add_argument("--deflection", type=float, default=45.0, help="as-built foil deflection, deg")
    ap.add_argument(
        "--params",
        default=str(ROOT / "tests" / "fixtures" / "20260909_1515_params_v4_rig_airmode.params"),
    )
    ap.add_argument("--forward", default="+z", help="Fusion axis that points forward (default +z)")
    ap.add_argument(
        "--up",
        default="+y",
        help="Fusion axis that points up (default +y; the dashboard display uses -y)",
    )
    ap.add_argument("--name", default="atlas_phase01_cad")
    ap.add_argument("--out", default=str(ROOT / "scenarios" / "atlas_phase01_cad.json"))
    ap.add_argument("--report", default=str(ROOT / "docs" / "cad_import_report.json"))
    args = ap.parse_args()

    frame = FusionFrame(forward=args.forward, up=args.up)
    dash = load_dashboard(args.html)
    template = Scenario.model_validate(json.loads(Path(args.template).read_text(encoding="utf-8")))
    edfs = order_edfs_px4(detect_edfs(dash), frame)
    print(f"{dash.doc}: {len(dash.bodies)} bodies, {len(edfs)} EDF ducts detected")

    ca = {e.name: e.value for e in read_params_file(args.params).entries}
    ref_cg, residuals = fit_reference_cg(edfs, ca, frame)
    print(
        f"reference CG fitted to {Path(args.params).name} (rotors 0..7), "
        f"Fusion mm: {np.round(ref_cg, 1).tolist()}"
    )
    print("residual CAD minus params per rotor, mm (FRD):")
    for i, r in enumerate(residuals):
        print(f"  rotor {i}: {np.round(r * 1e3, 1).tolist()}")

    weights = Weights.from_dashboard_export(args.weights) if args.weights else None
    created = datetime.now().astimezone().isoformat(timespec="seconds")
    scenario, report = build_scenario(
        dash,
        frame,
        template,
        args.name,
        created,
        weights=weights,
        reference_cg_fusion_mm=ref_cg,
        foils=not args.no_foils,
        foil_deflection_deg=args.deflection,
    )
    for foil in scenario.foils:
        pts = ", ".join(
            f"{i}: {list(foil.pressure_points_frd_m[i])}"
            for i in sorted(foil.pressure_points_frd_m)
        )
        print(
            f"foil {foil.id}: fans {foil.fan_ids}, deflection {foil.deflection_deg} deg; "
            f"pressure points FRD m: {pts}"
        )
    report["reference_cg_fusion_mm"] = np.round(ref_cg, 2).tolist()
    report["residuals_vs_params_mm"] = np.round(residuals * 1e3, 1).tolist()
    report["params_file"] = Path(args.params).name

    print("\nfans (FRD, relative to CG):")
    print(
        f"{'rotor':>5s} {'cad body':28s} {'x m':>8s} {'y m':>8s} {'z m':>8s}  "
        "cad axis (FRD)          cad tilt/az   scenario tilt/az"
    )
    for f, cad in zip(scenario.fans, report["fans"], strict=True):
        x, y, z = f.pos_frd_m
        print(
            f"{f.id:5d} {cad['cad_body']:28s} {x:8.4f} {y:8.4f} {z:8.4f}  "
            f"{str(cad['cad_axis_frd']):24s} "
            f"{cad['cad_tilt_deg']:5.1f}/{cad['cad_azimuth_deg']:5.1f}   "
            f"{f.tilt_deg:5.1f}/{f.azimuth_deg:5.1f}"
        )
    print(
        f"\nmass: total {scenario.mass.total_kg:.3f} kg, cg FRD {list(scenario.mass.cg_frd_m)}, "
        f"estimated={scenario.mass.estimated}"
    )
    if weights is None:
        print("no weights given: mass and CG are placeholders (see scenario.mass.notes)")
    elif report.get("missing_weights"):
        print("bodies without weight:", report["missing_weights"])

    Path(args.out).write_text(
        json.dumps(scenario.model_dump(mode="json"), indent=2) + "\n", encoding="utf-8"
    )
    Path(args.report).write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print(f"\nwrote {args.out} and {args.report}")


if __name__ == "__main__":
    main()
