"""Give a scenario landing gear that parks the airframe at a chosen pitch.

    uv run --project backend python scripts/add_landing_gear.py scenarios/<name>.json \
        --ground-pitch -20 --front 0.50,0.06 --rear -0.30,0.40 [--clearance 0.05] [--name <new>]

Reads the scenario and its GLB, fits the skin with collision boxes (vectra/core/gear.py),
designs four straight struts (a mirrored front pair and a mirrored rear pair at the given
FRD x,y in metres, y positive to the right) whose ball feet are the only points touching flat
ground at --ground-pitch (nose-up positive), writes frame.ground_pitch_deg and frame.gear into
the scenario and bakes the legs into the GLB as "gear:<leg>" nodes so the viewer, the STL and
the Gazebo export show them. In place unless --name gives a new scenario name. Prints one line.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))

from vectra.core.gear import (  # noqa: E402
    collision_boxes,
    design_gear,
    footprint_margin_m,
    leg_meshes,
    rest_height_m,
)
from vectra.scenario import Scenario  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def xy(text: str) -> tuple[float, float]:
    x, y = (float(v) for v in text.split(","))
    return x, abs(y)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("scenario", type=Path)
    ap.add_argument(
        "--ground-pitch", type=float, required=True, help="rest pitch, deg, nose-up positive"
    )
    ap.add_argument(
        "--front", type=xy, required=True, help="front pair x,y in FRD metres (mirrored in y)"
    )
    ap.add_argument(
        "--rear", type=xy, required=True, help="rear pair x,y in FRD metres (mirrored in y)"
    )
    ap.add_argument(
        "--clearance", type=float, default=0.05, help="ground clearance under the skin, m"
    )
    ap.add_argument(
        "--name", help="write a new scenario and GLB under this name instead of in place"
    )
    args = ap.parse_args()

    import trimesh

    scenario = Scenario.model_validate(json.loads(args.scenario.read_text(encoding="utf-8")))
    if not scenario.meta.cad_model:
        sys.exit("scenario has no cad_model GLB; the gear is designed against the CAD skin")
    glb = ROOT / "scenarios" / scenario.meta.cad_model
    scene = trimesh.load(str(glb), force="scene")
    for name in [n for n in scene.geometry if str(n).startswith("gear:")]:
        scene.delete_geometry(name)  # previous gear, replaced below
    skin = np.vstack([g.vertices for g in scene.geometry.values()])
    boxes = collision_boxes(skin)
    fx, fy = args.front
    rx, ry = args.rear
    feet = {"front_r": (fx, fy), "front_l": (fx, -fy), "rear_r": (rx, ry), "rear_l": (rx, -ry)}
    legs = design_gear(skin, args.ground_pitch, feet, boxes, clearance_m=args.clearance)

    name = args.name or scenario.meta.name
    frame = scenario.frame.model_copy(update={"ground_pitch_deg": args.ground_pitch, "gear": legs})
    note = (
        f"Landing gear added: parks at {args.ground_pitch:g} deg pitch on four legs "
        f"(front pair x {fx:+.2f} y ±{fy:.2f}, rear pair x {rx:+.2f} y ±{ry:.2f} m FRD), "
        f"{args.clearance:g} m skin clearance."
    )
    meta = scenario.meta.model_copy(
        update={
            "name": name,
            "cad_model": f"{name}.glb",
            "notes": f"{scenario.meta.notes} | {note}".strip(" |"),
        }
    )
    scenario = scenario.model_copy(update={"frame": frame, "meta": meta})

    for node, mesh in leg_meshes(legs).items():
        scene.add_geometry(mesh, node_name=node, geom_name=node)
    out_glb = ROOT / "scenarios" / f"{name}.glb"
    scene.export(str(out_glb), file_type="glb")
    out_json = ROOT / "scenarios" / f"{name}.json"
    out_json.write_text(
        json.dumps(scenario.model_dump(mode="json"), indent=2) + "\n", encoding="utf-8"
    )

    lengths = ", ".join(
        f"{leg.name} {float(np.linalg.norm(np.subtract(leg.foot_frd_m, leg.attach_frd_m))):.3f} m"
        for leg in legs
    )
    print(
        f"{out_json.name}: {len(boxes)} skin boxes; legs {lengths};"
        f" CG {rest_height_m(scenario):.3f} m above ground when parked;"
        f" footprint margin {footprint_margin_m(scenario):.3f} m;"
        f" GLB {out_glb.name} ({out_glb.stat().st_size / 1e6:.1f} MB)"
    )


if __name__ == "__main__":
    main()
