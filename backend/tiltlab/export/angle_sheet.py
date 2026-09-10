"""Foil design sheet: what to model in the CAD for each motor's chosen foil deflection.

For every motor that blows into a foil the sheet gives the deflection, the change from the
as-built foil, the jet exit direction and the thrust direction in both FRD and the CAD frame,
and the CAD coordinates of the motor centre, the duct exit and the pressure point. Angles are
in degrees, CAD coordinates in the scenario's cad_units (mm for the Fusion dashboard).

CAD frame mapping comes from Scenario.frame (cad_forward_axis, cad_up_axis, cad_origin): a
vector v_frd maps to v_cad = R^T v_frd with R the FRD-from-CAD rotation of tiltlab.cad
.fusion_dashboard.FusionFrame, and a point p_frd (m) maps to cad_origin + R^T p_frd * scale.
"""

from __future__ import annotations

import csv
import math
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from tiltlab.export.naming import timestamped_name
from tiltlab.scenario import Scenario, deflect_axis

AS_BUILT_DEFLECTION_DEG = 45.0
UNIT_SCALE = {"mm": 1000.0, "m": 1.0, "in": 39.37007874015748}


def _axis_unit(spec: str) -> np.ndarray:
    sign = -1.0 if spec.strip().startswith("-") else 1.0
    e = np.zeros(3)
    e["XYZ".index(spec.strip().lstrip("+-").upper())] = sign
    return e


def cad_rotation(scenario: Scenario) -> np.ndarray:
    """R with v_frd = R @ v_cad (rows: forward, right, down expressed in CAD axes)."""
    fwd = _axis_unit(scenario.frame.cad_forward_axis)
    down = -_axis_unit(scenario.frame.cad_up_axis)
    right = np.cross(down, fwd)
    return np.vstack([fwd, right, down])


def _fmt_dir(v: np.ndarray) -> str:
    vals = [0.0 if abs(float(x)) < 5e-4 else float(x) for x in v]  # no "-0.000"
    return "(" + ", ".join(f"{x:.3f}" for x in vals) + ")"


def foil_angle_sheet(scenario: Scenario) -> list[dict[str, Any]]:
    """One row per foil motor (rotor id order)."""
    R = cad_rotation(scenario)
    scale = UNIT_SCALE[scenario.frame.cad_units]
    origin = np.asarray(scenario.frame.cad_origin or (0.0, 0.0, 0.0), dtype=float)
    units = scenario.frame.cad_units

    def to_cad_point(p_frd_m: np.ndarray) -> list[float]:
        return [round(float(x), 2) for x in origin + R.T @ (np.asarray(p_frd_m) * scale)]

    def to_cad_dir(v_frd: np.ndarray) -> np.ndarray:
        return R.T @ np.asarray(v_frd)

    rows: list[dict[str, Any]] = []
    for fan in scenario.fans_sorted():
        foil = scenario.foil_for(fan.id)
        if foil is None:
            continue
        d = foil.deflection_for(fan.id)
        d_eff = foil.effective_deflection_for(fan.id)
        attached = foil.attached(fan.id)
        motor_axis = fan.axis()  # thrust direction of the bare motor; exhaust is the opposite
        thrust = deflect_axis(motor_axis, d_eff)
        exhaust = -thrust
        pressure = scenario.effective_pos(fan)
        motor = np.asarray(fan.pos_frd_m, dtype=float)
        duct_exit = motor - motor_axis * 0.065  # aft end of a 130 mm duct
        # angle of the exhaust below the fore-aft axis (positive down), in the vertical plane
        exit_down_deg = math.degrees(math.atan2(float(exhaust[2]), float(-exhaust[0])))
        rows.append(
            {
                "rotor": fan.id,
                "output": fan.output,
                "foil": foil.id,
                "side": "left" if fan.pos_frd_m[1] < 0 else "right",
                "deflection_deg": round(d, 2),
                "effective_turning_deg": round(d_eff, 2),
                "jet_attached": attached,
                "coanda_limit_deg": (
                    round(foil.coanda.separation_deg(), 1)
                    if foil.coanda and foil.coanda.enabled
                    else None
                ),
                "coanda_radius_m": foil.coanda.radius_m if foil.coanda else None,
                "change_from_as_built_deg": round(d - AS_BUILT_DEFLECTION_DEG, 2),
                "exhaust_angle_below_fore_aft_deg": round(exit_down_deg, 2),
                "thrust_dir_frd": _fmt_dir(thrust),
                "thrust_dir_cad": _fmt_dir(to_cad_dir(thrust)),
                "exhaust_dir_frd": _fmt_dir(exhaust),
                "exhaust_dir_cad": _fmt_dir(to_cad_dir(exhaust)),
                f"motor_centre_cad_{units}": to_cad_point(motor),
                f"duct_exit_cad_{units}": to_cad_point(duct_exit),
                f"pressure_point_cad_{units}": to_cad_point(pressure),
                "pressure_point_frd_m": [round(float(x), 4) for x in pressure],
                "thrust_scale": round(scenario.foil_ct_scale(fan), 4),
                "ct_effective_N": round(scenario.fan_ct_effective(fan), 3),
            }
        )
    return rows


def angle_sheet_markdown(scenario: Scenario, rows: list[dict[str, Any]]) -> str:
    units = scenario.frame.cad_units
    fwd, up = scenario.frame.cad_forward_axis, scenario.frame.cad_up_axis
    lines = [
        f"# Foil design sheet: {scenario.meta.name}",
        "",
        f"CAD frame: forward {fwd}, up {up}, units {units}; CAD origin of the FRD frame at "
        f"{list(scenario.frame.cad_origin) if scenario.frame.cad_origin else 'unknown'} {units}.",
        "Deflection is the angle the foil turns the jet down from the motor axis (0 straight aft, "
        "90 straight down, 180 straight forward). Model each foil segment so its exit plane sends "
        "the jet along exhaust_dir_cad; the change column is the rotation to apply to the as-built "
        f"{AS_BUILT_DEFLECTION_DEG:g} degree foil about the lateral axis (positive turns the exit "
        "further down and forward).",
        "",
        "| rotor | side | wrap | effective turning | attached | change vs as built | "
        f"exhaust below fore-aft | exhaust dir CAD | pressure point CAD {units} | "
        f"motor centre CAD {units} |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['rotor']} | {r['side']} | {r['deflection_deg']:g} | "
            f"{r['effective_turning_deg']:g} | {'yes' if r['jet_attached'] else 'NO'} | "
            f"{r['change_from_as_built_deg']:+g} | {r['exhaust_angle_below_fore_aft_deg']:g} | "
            f"{r['exhaust_dir_cad']} | {r[f'pressure_point_cad_{units}']} | "
            f"{r[f'motor_centre_cad_{units}']} |"
        )
    return "\n".join(lines) + "\n"


def export_angle_sheet(
    scenario: Scenario, out_dir: str | Path, now: datetime | None = None
) -> tuple[Path, Path]:
    """Write <stamp>_<scenario>_foil_sheet.csv and .md; returns both paths."""
    rows = foil_angle_sheet(scenario)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stem = f"{scenario.meta.name}_foil_sheet"
    csv_path = out / timestamped_name(stem, "csv", now)
    md_path = out / timestamped_name(stem, "md", now)
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            for r in rows:
                writer.writerow(
                    {
                        k: (v if not isinstance(v, list) else " ".join(map(str, v)))
                        for k, v in r.items()
                    }
                )
    else:
        csv_path.write_text("", encoding="utf-8")
    md_path.write_text(angle_sheet_markdown(scenario, rows), encoding="utf-8")
    return csv_path, md_path
