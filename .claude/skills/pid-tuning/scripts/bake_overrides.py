"""Bake tuned PX4 parameters into a scenario's control.px4_params_override and re-export its
Gazebo harness, so a relaunch reproduces what was tuned live.

    uv run --project backend python .claude/skills/pid-tuning/scripts/bake_overrides.py \
        scenarios/<name>.json NAME VALUE [NAME VALUE ...] [--note "why"] [--no-export]

Values are parsed as numbers. CA_* entries are allowed (scenario_to_ca_params applies them last,
so the board push and the airframe both carry them); the metrics keep using the geometry. The
scenario file is rewritten in place with the note appended to meta.notes, and the harness lands
in exports/gazebo/<name>/ (overwriting an export of the same name).
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import date
from pathlib import Path

from vectra.scenario import Scenario


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario")
    ap.add_argument("pairs", nargs="*", help="NAME VALUE ...")
    ap.add_argument("--note", default="")
    ap.add_argument("--no-export", action="store_true")
    args = ap.parse_args()
    if len(args.pairs) % 2:
        ap.error("NAME VALUE pairs expected")
    path = Path(args.scenario)
    d = json.loads(path.read_text(encoding="utf-8"))
    over = d.setdefault("control", {}).setdefault("px4_params_override", {})
    changed = {}
    for name, raw in zip(args.pairs[::2], args.pairs[1::2], strict=True):
        v = float(raw)
        v = int(v) if v.is_integer() and name.split("_")[-1] not in ("P", "I", "D", "K") else v
        changed[name] = (over.get(name), v)
        over[name] = v
    stamp = date.today().isoformat()
    note = f" | Overrides baked {stamp}: " + ", ".join(f"{k} {v[1]}" for k, v in changed.items())
    if args.note:
        note += f" ({args.note})"
    d["meta"]["notes"] = (d["meta"].get("notes", "") + note).strip(" |")
    sc = Scenario.model_validate(d)  # validates before anything is written
    path.write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")
    for k, (old, new) in changed.items():
        print(f"{k}: {old} -> {new}")
    if not args.no_export:
        from vectra.export.gazebo import export_gazebo

        root = export_gazebo(sc, Path("exports/gazebo"))["root"]
        print(f"exported {root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
