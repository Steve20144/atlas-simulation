"""Build backend/vectra/px4/param_catalog.json from the pinned PX4 tree.

Reads every PARAM_DEFINE_* block (doc comment tags @unit @min @max @value @boolean
@reboot_required @group) in *.c files and every ``parameters:`` group in module.yaml files
under third_party/PX4-Autopilot (sparse checkout, so the catalogue covers the modules that
are checked out; the UI accepts any other name as free text). ``${i}`` templates are
expanded over num_instances. Run after scripts/fetch_px4.sh:

    uv run --project backend python scripts/build_param_catalog.py
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
PX4 = ROOT / "third_party" / "PX4-Autopilot"
OUT = ROOT / "backend" / "vectra" / "px4" / "param_catalog.json"

_DEFINE = re.compile(
    r"/\*\*(?P<doc>(?:(?!\*/).)*)\*/\s*PARAM_DEFINE_(?P<ctype>INT32|FLOAT)\(\s*(?P<name>[A-Z0-9_]+)\s*,"
    r"\s*(?P<default>[^)]*)\)",
    re.S,
)
_TAG = re.compile(r"^@(\w+)\s*(.*)$")


def _num(text: str) -> float | None:
    text = text.strip().rstrip("fF")
    try:
        return float(text)
    except ValueError:
        return None


def _entry(name: str, ptype: str, group: str, source: str) -> dict[str, Any]:
    return {
        "name": name, "short": "", "long": "", "type": ptype, "unit": None, "min": None,
        "max": None, "default": None, "values": None, "group": group, "reboot": False,
        "source": source,
    }


def parse_c(path: Path) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    text = path.read_text(encoding="utf-8", errors="replace")
    for m in _DEFINE.finditer(text):
        lines = [ln.strip().lstrip("*").strip() for ln in m.group("doc").splitlines()]
        e = _entry(m.group("name"), m.group("ctype").lower(), "", str(path.relative_to(PX4)))
        e["default"] = _num(m.group("default"))
        prose: list[str] = []
        for ln in lines:
            tag = _TAG.match(ln)
            if not tag:
                prose.append(ln)
                continue
            key, val = tag.group(1), tag.group(2).strip()
            if key == "unit":
                e["unit"] = val
            elif key in ("min", "max"):
                e[key] = _num(val)
            elif key == "value":
                code, _, label = val.partition(" ")
                e["values"] = {**(e["values"] or {}), code: label.strip()}
            elif key == "boolean":
                e["type"] = "boolean"
            elif key == "reboot_required":
                e["reboot"] = val.lower() == "true"
            elif key == "group":
                e["group"] = val
        paragraphs = [p for p in "\n".join(prose).strip().split("\n\n") if p.strip()]
        e["short"] = paragraphs[0].replace("\n", " ").strip() if paragraphs else e["name"]
        e["long"] = "\n\n".join(p.strip() for p in paragraphs[1:])
        if e["values"] and e["type"] == "int32":
            e["type"] = "enum"
        out.append(e)
    return out


def parse_yaml(path: Path) -> list[dict[str, Any]]:
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    out: list[dict[str, Any]] = []
    for grp in doc.get("parameters") or []:
        group = str(grp.get("group", ""))
        for tmpl, d in (grp.get("definitions") or {}).items():
            n = int(d.get("num_instances", 1))
            start = int(d.get("instance_start", 0))
            for k in range(n):
                i = start + k
                name = tmpl.replace("${i}", str(i))
                e = _entry(name, str(d.get("type", "float")), group, str(path.relative_to(PX4)))
                desc = d.get("description") or {}
                e["short"] = str(desc.get("short", name)).replace("${i}", str(i)).strip()
                e["long"] = str(desc.get("long", "")).replace("${i}", str(i)).strip()
                e["unit"] = d.get("unit")
                e["min"], e["max"] = d.get("min"), d.get("max")
                default = d.get("default")
                if isinstance(default, list):
                    default = default[k] if k < len(default) else default[-1]
                e["default"] = default if isinstance(default, (int, float)) else _num(str(default))
                if isinstance(d.get("values"), dict):
                    e["values"] = {str(c): str(label) for c, label in d["values"].items()}
                if isinstance(d.get("bit"), dict):
                    e["values"] = {str(c): str(label) for c, label in d["bit"].items()}
                e["reboot"] = bool(d.get("reboot_required", False))
                out.append(e)
    return out


# Parameters Vectra writes or reads that live in modules outside the sparse checkout (rc_update,
# systemcmds, pwm_out). Descriptions follow the PX4 v1.17.0 parameter reference.
_RC_CAL = "Radio Calibration"
SUPPLEMENT: list[dict[str, Any]] = [
    {"name": "SYS_HITL", "short": "Enable HITL/SIH mode on next boot", "type": "enum",
     "values": {"0": "Disabled", "1": "HITL (hardware in the loop)",
                "2": "SIH (simulation in hardware)"},
     "default": 0, "reboot": True, "group": "System"},
    {"name": "SYS_AUTOSTART", "short": "Auto-start script index (airframe)", "type": "int32",
     "min": 0, "max": 9999999, "default": 0, "reboot": True, "group": "System"},
    {"name": "RC_MAP_ROLL", "short": "Roll control channel (0 unassigned)", "type": "int32",
     "min": 0, "max": 18, "default": 0, "group": _RC_CAL},
    {"name": "RC_MAP_PITCH", "short": "Pitch control channel (0 unassigned)", "type": "int32",
     "min": 0, "max": 18, "default": 0, "group": _RC_CAL},
    {"name": "RC_MAP_THROTTLE", "short": "Throttle control channel (0 unassigned)",
     "type": "int32", "min": 0, "max": 18, "default": 0, "group": _RC_CAL},
    {"name": "RC_MAP_YAW", "short": "Yaw control channel (0 unassigned)", "type": "int32",
     "min": 0, "max": 18, "default": 0, "group": _RC_CAL},
    {"name": "RC_MAP_ARM_SW", "short": "Arm switch channel (0 unassigned)", "type": "int32",
     "min": 0, "max": 18, "default": 0, "group": "Radio Switches"},
    {"name": "RC_MAP_KILL_SW", "short": "Emergency kill switch channel (0 unassigned)",
     "type": "int32", "min": 0, "max": 18, "default": 0, "group": "Radio Switches"},
    {"name": "RC3_MIN", "short": "RC channel 3 minimum", "type": "float", "unit": "us",
     "min": 800, "max": 1500, "default": 1000, "group": _RC_CAL},
    {"name": "RC3_TRIM", "short": "RC channel 3 trim", "type": "float", "unit": "us",
     "min": 800, "max": 2200, "default": 1500, "group": _RC_CAL},
    {"name": "RC3_MAX", "short": "RC channel 3 maximum", "type": "float", "unit": "us",
     "min": 1500, "max": 2200, "default": 2000, "group": _RC_CAL},
    {"name": "RC3_REV", "short": "RC channel 3 reverse", "type": "enum",
     "values": {"-1": "Reverse", "1": "Normal"}, "default": 1, "group": _RC_CAL},
]


def supplement() -> list[dict[str, Any]]:
    out = []
    for d in SUPPLEMENT:
        e = _entry(d["name"], d["type"], d.get("group", ""), "curated (module not checked out)")
        e.update({k: v for k, v in d.items() if k not in ("name", "type", "group")})
        out.append(e)
    return out


def main() -> int:
    if not PX4.exists():
        print(f"missing {PX4}; run scripts/fetch_px4.sh first", file=sys.stderr)
        return 1
    entries: dict[str, dict[str, Any]] = {}
    for p in sorted(PX4.rglob("*.c")):
        for e in parse_c(p):
            entries.setdefault(e["name"], e)
    for p in sorted(PX4.rglob("module.yaml")):
        for e in parse_yaml(p):
            entries.setdefault(e["name"], e)
    for e in supplement():
        entries.setdefault(e["name"], e)
    data = {
        "px4": "v1.17.0 d6f12ad1c4f70ad3230afd7d86e971421e02fef4 (sparse checkout)",
        "count": len(entries),
        "params": [entries[k] for k in sorted(entries)],
    }
    OUT.write_text(json.dumps(data, indent=1) + "\n", encoding="utf-8")
    print(f"{len(entries)} parameters -> {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
