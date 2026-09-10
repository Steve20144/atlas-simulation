"""PX4 .params export (PLAN.md M9) and the matching import.

The export merges only the CA_* geometry of the Scenario (CA_ROTOR_COUNT and
CA_ROTORn_{PX,PY,PZ,AX,AY,AZ,CT,KM} for the ten fans) plus the concept-specific set onto the
user's full parameter backup, so every other line of that backup is preserved byte for byte
(CRLF, original value text). Units in the written values: PX/PY/PZ metres FRD relative to the
CG, AX/AY/AZ FRD thrust direction (unit vector), CT newtons at full command, KM dimensionless.

Concept sets:
- stock: CA_METHOD left exactly as in the base file.
- fully_actuated: CA_METHOD = 0 (pseudo-inverse), FD_FAIL_P = 0, FD_FAIL_R = 0, following the
  PX4 omnicopter airframe guidance (attitude failure detection must be off when the vehicle is
  meant to hold large tilt angles).
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Literal

import numpy as np

from tiltlab.core.fan import FanCurve as FanCurveModel
from tiltlab.core.geometry import rotors_from_scenario
from tiltlab.core.params_px4 import (
    ROTOR_FIELDS,
    ParamFile,
    apply_ca_params,
    read_params_file,
    scenario_from_ca_params,
    write_params_file,
)
from tiltlab.export.naming import has_timestamp_prefix, timestamped_name
from tiltlab.scenario import NUM_FANS, Scenario

Concept = Literal["stock", "fully_actuated"]

REPO_ROOT = Path(__file__).resolve().parents[3]
FIXTURES_DIR = REPO_ROOT / "tests" / "fixtures"
COLUMN_HEADER = "# Vehicle-Id Component-Id Name Value Type"

CONCEPT_PARAMS: dict[str, dict[str, int]] = {
    "stock": {},
    "fully_actuated": {"CA_METHOD": 0, "FD_FAIL_P": 0, "FD_FAIL_R": 0},
}


def latest_backup_params(search_dir: str | Path = FIXTURES_DIR) -> Path:
    """The user's most recent full parameter backup in `search_dir`.

    Files named with the app's 'YYYYMMDD_HHMM_' prefix win by name (newest stamp); otherwise
    the newest modification time decides.
    """
    files = sorted(Path(search_dir).glob("*.params"))
    if not files:
        raise FileNotFoundError(f"no .params backup in {search_dir}")
    stamped = [p for p in files if has_timestamp_prefix(p)]
    if stamped:
        return max(stamped, key=lambda p: p.name)
    return max(files, key=lambda p: (p.stat().st_mtime, p.name))


def fan_curve_ct(scenario: Scenario, curve_ref: str) -> float:
    """CA_ROTORn_CT (N) for a fan curve: thrust at cmd 1.0 via core.fan.FanCurve.ct_for_px4."""
    curve = scenario.fan_curves[curve_ref]
    return FanCurveModel.from_scenario_curve(curve.model_dump()).ct_for_px4()


def ca_geometry_params(scenario: Scenario) -> dict[str, int | float]:
    """CA_ROTOR_COUNT and CA_ROTORn_* for the ten fans, float32-rounded like PX4 stores them.

    CT comes from the fan curve at cmd 1.0 (core.fan) unless control.px4_params_override has
    CA_ROTOR<n>_CT; KM is the signed km when control.reaction_torque is on, else 0. Other
    CA_ROTOR* overrides are applied last. CA_METHOD is deliberately not included (concept set).
    """
    params: dict[str, int | float] = {"CA_ROTOR_COUNT": NUM_FANS}
    fans = scenario.fans_sorted()
    for fan, rotor in zip(fans, rotors_from_scenario(scenario), strict=True):
        ct_key = f"CA_ROTOR{fan.id}_CT"
        ct = scenario.control.px4_params_override.get(
            ct_key, fan_curve_ct(scenario, fan.curve_ref) * scenario.foil_ct_scale(fan)
        )
        vals = (*rotor.position, *rotor.axis, float(ct), scenario.fan_km(fan))
        for key, v in zip(ROTOR_FIELDS, vals, strict=True):
            params[f"CA_ROTOR{fan.id}_{key}"] = float(np.float32(v))
    for key, v in scenario.control.px4_params_override.items():
        if key.startswith("CA_ROTOR") and key != "CA_ROTOR_COUNT":
            params[key] = float(np.float32(v))
    return params


def concept_params(concept: Concept) -> dict[str, int]:
    if concept not in CONCEPT_PARAMS:
        raise ValueError(f"unknown concept {concept!r}; expected one of {sorted(CONCEPT_PARAMS)}")
    return dict(CONCEPT_PARAMS[concept])


def export_header_lines(
    scenario: Scenario, concept: Concept, base_params_path: Path, now: datetime | None = None
) -> list[str]:
    """Comment block: tilt and azimuth per fan, the fan curve used and its estimated flag."""
    stamp = (now or datetime.now()).isoformat(timespec="minutes")
    lines = [
        "#",
        f"# tiltlab export {stamp}: scenario '{scenario.meta.name}', concept {concept}",
        f"# base backup: {base_params_path.name} (only CA_ROTOR*/concept lines changed)",
        f"# reaction torque (KM): {'on' if scenario.control.reaction_torque else 'off, KM = 0'}",
        "# fan  tilt_deg  azimuth_deg  pos_frd_m (x, y, z)  curve",
    ]
    for f in scenario.fans_sorted():
        x, y, z = f.pos_frd_m
        lines.append(
            f"# {f.id:>3}  {f.tilt_deg:8.3f}  {f.azimuth_deg:11.3f}  "
            f"({x:.4f}, {y:.4f}, {z:.4f})  {f.curve_ref}"
        )
    for ref, curve in scenario.fan_curves.items():
        est = "ESTIMATED (manufacturer max only)" if curve.estimated else "measured"
        ct = fan_curve_ct(scenario, ref)
        lines.append(f"# fan curve {ref}: CT = {ct:.4g} N at cmd 1.0, {est}")
    if concept == "fully_actuated":
        lines.append("# fully_actuated: CA_METHOD 0, FD_FAIL_P 0, FD_FAIL_R 0; needs a patched PX4")
    lines.append("#")
    return lines


def merged_param_file(
    scenario: Scenario, concept: Concept, base: ParamFile, header_block: list[str]
) -> ParamFile:
    """Base file with the geometry and concept sets applied and the tiltlab header inserted
    before the column-header comment (or appended when the base has none)."""
    params: dict[str, int | float] = {**ca_geometry_params(scenario), **concept_params(concept)}
    out = apply_ca_params(base, params)
    header = list(base.header)
    idx = next((i for i, h in enumerate(header) if h.strip() == COLUMN_HEADER), len(header))
    out.header = header[:idx] + header_block + header[idx:]
    return out


def export_params(
    scenario: Scenario,
    concept: Concept | None = None,
    base_params_path: str | Path | None = None,
    out_dir: str | Path = ".",
    now: datetime | None = None,
) -> Path:
    """Write '<stamp>_<scenario>_<concept>_px4.params' into out_dir and return its path.

    concept defaults to scenario.control.concept; base_params_path to latest_backup_params().
    """
    concept = concept or scenario.control.concept
    base_path = Path(base_params_path) if base_params_path else latest_backup_params()
    base = read_params_file(base_path)
    header_block = export_header_lines(scenario, concept, base_path, now)
    pf = merged_param_file(scenario, concept, base, header_block)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / timestamped_name(f"{scenario.meta.name}_{concept}_px4", "params", now)
    write_params_file(pf, path)
    return path


def import_params_to_scenario(path: str | Path, template_scenario: Scenario) -> Scenario:
    """Scenario from an exported (or any) .params file: geometry from CA_ROTORn_*, everything
    else (mass, fan curves, rig, control defaults, meta) from template_scenario.

    Round trip export -> import reproduces tilt, azimuth and positions to float32 resolution
    (the .params file stores float32), which is gate G6's local half.
    """
    params = read_params_file(path).to_dict()
    t = template_scenario
    return scenario_from_ca_params(
        params,
        name=t.meta.name,
        created=t.meta.created,
        fan_curves=t.fan_curves,
        curve_ref=t.fans_sorted()[0].curve_ref,
        mass=t.mass,
        rig=t.rig,
        control=t.control,
        notes=t.meta.notes,
        px4_version=t.meta.px4_version,
    )
