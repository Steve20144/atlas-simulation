"""PX4 parameter files, ULog parameter snapshots and the Scenario <-> CA_ROTORn_* mapping.

File format (QGroundControl "Onboard parameters" export): '#' comment header lines, then one
tab-separated record per parameter 'vehicle_id\tcomponent_id\tname\tvalue\ttype' with CRLF
line endings; type 6 = INT32, 9 = FLOAT. Float values are written the way Qt prints a float
with QLocale::FloatingPointShortest: the shortest digit string that round-trips in float32,
decimal notation when -4 < decpt <= digit count, otherwise exponent notation with a two-digit
exponent ('1.6e+03', '-1e+01', '1e-05'). This reproduces every value of the fixture files except
two entries (MPC_MAN_Y_MAX '30' in the v4 file and '60' in the v3 file) that QGC wrote in plain
decimal after the user edited them, while the same export writes 30 as '3e+01' elsewhere; the
text therefore depends on QGC state, not on the value. To round-trip byte for byte, each entry
keeps the text it was read with and reuses it as long as its value is unchanged.

Units in the mapping: CA_ROTORn_PX/PY/PZ metres FRD relative to the CG; AX/AY/AZ FRD thrust
direction; CT newtons at full command; KM dimensionless (positive CCW, module.yaml:217-218).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from tiltlab.scenario import (
    NUM_FANS,
    Control,
    CurvePoint,
    Fan,
    FanCurve,
    Mass,
    Meta,
    Rig,
    Scenario,
    axis_to_tilt_azimuth,
)

if TYPE_CHECKING:
    from collections.abc import Iterable

PARAM_TYPE_INT32 = 6
PARAM_TYPE_FLOAT = 9
ROTOR_FIELDS = ("PX", "PY", "PZ", "AX", "AY", "AZ", "CT", "KM")


@dataclass
class ParamEntry:
    vehicle_id: int
    component_id: int
    name: str
    value: int | float
    type_code: int
    raw: str | None = None  # value text as read from a file, reused while the value is unchanged

    def value_text(self) -> str:
        if self.raw is not None and parse_param_value(self.raw, self.type_code) == self.value:
            return self.raw
        return format_param_value(self.value, self.type_code)


@dataclass
class ParamFile:
    header: list[str] = field(default_factory=list)
    entries: list[ParamEntry] = field(default_factory=list)

    def to_dict(self) -> dict[str, int | float]:
        return {e.name: e.value for e in self.entries}


def format_px4_float(value: float) -> str:
    """Format a float32 value the way the QGC parameter export does (see module docstring)."""
    v = np.float32(value)
    if np.isnan(v):
        return "nan"
    if np.isinf(v):
        return "inf" if v > 0 else "-inf"
    if v == 0:
        return "0"
    sci = np.format_float_scientific(v, unique=True, trim="-")  # e.g. '-5.2359875e-04'
    mantissa, exp_s = sci.split("e")
    exp10 = int(exp_s)
    sign = "-" if mantissa.startswith("-") else ""
    digits = mantissa.lstrip("-").replace(".", "")
    decpt = exp10 + 1  # position of the decimal point relative to the digit string
    n = len(digits)
    if -4 < decpt <= n:
        if decpt <= 0:
            body = "0." + "0" * (-decpt) + digits
        elif decpt == n:
            body = digits
        else:
            body = digits[:decpt] + "." + digits[decpt:]
    else:
        body = digits[0] + ("." + digits[1:] if n > 1 else "")
        body += f"e{'+' if exp10 >= 0 else '-'}{abs(exp10):02d}"
    return sign + body


def format_param_value(value: int | float, type_code: int) -> str:
    if type_code == PARAM_TYPE_INT32:
        return str(int(value))
    return format_px4_float(float(value))


def parse_param_value(text: str, type_code: int) -> int | float:
    if type_code == PARAM_TYPE_INT32:
        return int(text)
    return float(text)


def read_params_file(path: str | Path) -> ParamFile:
    """Read a .params file (see module docstring). Header comment lines are kept verbatim."""
    raw = Path(path).read_bytes().decode("utf-8")
    pf = ParamFile()
    for line in raw.split("\r\n" if "\r\n" in raw else "\n"):
        if not line:
            continue
        if line.startswith("#"):
            pf.header.append(line)
            continue
        parts = line.split("\t")
        if len(parts) != 5:
            raise ValueError(f"malformed parameter line: {line!r}")
        type_code = int(parts[4])
        pf.entries.append(
            ParamEntry(
                int(parts[0]),
                int(parts[1]),
                parts[2],
                parse_param_value(parts[3], type_code),
                type_code,
                raw=parts[3],
            )
        )
    return pf


def params_file_to_bytes(pf: ParamFile) -> bytes:
    lines = list(pf.header)
    for e in pf.entries:
        lines.append(f"{e.vehicle_id}\t{e.component_id}\t{e.name}\t{e.value_text()}\t{e.type_code}")
    return ("\r\n".join(lines) + "\r\n").encode("utf-8")


def write_params_file(pf: ParamFile, path: str | Path) -> None:
    Path(path).write_bytes(params_file_to_bytes(pf))


def default_header(px4_version: str = "1.17.0", note: str | None = None) -> list[str]:
    header = [
        "# Onboard parameters for Vehicle 1",
        "#",
        "# Stack: PX4 Pro",
        "# Vehicle: Multi-Rotor",
        f"# Version: {px4_version} ",
        "# Git Revision: d6f12ad1c4000000",
        "#",
    ]
    if note:
        header.append(f"# {note}")
        header.append("#")
    header.append("# Vehicle-Id Component-Id Name Value Type")
    return header


def params_file_from_dict(
    params: dict[str, int | float],
    types: dict[str, int] | None = None,
    header: list[str] | None = None,
) -> ParamFile:
    """Build a ParamFile from a name -> value dict. The type comes from `types` when given,
    otherwise from the Python type (int -> INT32, float -> FLOAT). Entries are sorted by name."""
    pf = ParamFile(header=list(header) if header is not None else default_header())
    for name in sorted(params):
        value = params[name]
        default_code = PARAM_TYPE_INT32 if isinstance(value, int | np.integer) else PARAM_TYPE_FLOAT
        code = (types or {}).get(name, default_code)
        typed: int | float = int(value) if code == PARAM_TYPE_INT32 else float(value)
        pf.entries.append(ParamEntry(1, 1, name, typed, code))
    return pf


def params_from_ulog(path: str | Path) -> dict[str, int | float]:
    """Parameter snapshot at log start (ULog.initial_parameters via pyulog), same dict form as
    ParamFile.to_dict(): INT32 as int, FLOAT as float."""
    from pyulog import ULog

    ulog = ULog(str(path), message_name_filter_list=[])
    out: dict[str, int | float] = {}
    for name, value in ulog.initial_parameters.items():
        out[name] = int(value) if isinstance(value, int | np.integer) else float(value)
    return out


# ----------------------------------------------------------------------------------------
# Scenario <-> CA parameters
# ----------------------------------------------------------------------------------------


def scenario_to_ca_params(scenario: Scenario) -> dict[str, int | float]:
    """CA_ROTOR_COUNT, CA_METHOD and CA_ROTORn_{PX,PY,PZ,AX,AY,AZ,CT,KM} for n = 0..9 derived
    from the Scenario: position = pos_frd_m - cg_frd_m (m, FRD), axis from tilt/azimuth,
    CT = fan curve thrust at cmd 1.0 (N), KM = signed km if control.reaction_torque else 0.
    Entries of control.px4_params_override that start with 'CA_' are applied last."""
    from tiltlab.core.geometry import rotors_from_scenario

    params: dict[str, int | float] = {
        "CA_ROTOR_COUNT": NUM_FANS,
        "CA_METHOD": int(scenario.control.ca_method),
    }
    for i, rotor in enumerate(rotors_from_scenario(scenario)):
        p = f"CA_ROTOR{i}_"
        vals = (*rotor.position, *rotor.axis, rotor.thrust_coef, rotor.moment_ratio)
        for key, v in zip(ROTOR_FIELDS, vals, strict=True):
            params[p + key] = float(np.float32(v))
    for key, v in scenario.control.px4_params_override.items():
        if key.startswith("CA_"):
            params[key] = (
                int(v)
                if key in ("CA_METHOD", "CA_ROTOR_COUNT", "CA_R_REV", "CA_AIRFRAME")
                else float(v)
            )
    return params


def scenario_from_ca_params(
    params: dict[str, int | float],
    *,
    name: str,
    created: str,
    fan_curves: dict[str, FanCurve],
    curve_ref: str,
    mass: Mass,
    rig: Rig | None = None,
    control: Control | None = None,
    notes: str = "",
    px4_version: str = "1.17.0",
) -> Scenario:
    """Scenario from CA_ROTORn_* parameters (the inverse of scenario_to_ca_params).

    Positions become pos_frd_m (cg assumed at the origin unless mass.cg_frd_m says otherwise,
    in which case pos_frd_m = P + cg). Tilt/azimuth from the normalised axis. KM sign sets the
    spin (positive -> CCW); km = |KM|; control.reaction_torque is on if any KM != 0. When the
    CT of a rotor differs from the fan curve's thrust at cmd 1.0 by more than float32
    resolution, CA_ROTORn_CT is recorded in control.px4_params_override so the round trip is
    exact and the flown CT is preserved.
    """
    from tiltlab.core.geometry import rotors_from_px4_params

    rotors = rotors_from_px4_params(params)
    if len(rotors) != NUM_FANS:
        raise ValueError(f"expected CA_ROTOR_COUNT = {NUM_FANS}, got {len(rotors)}")
    ctrl = (control or Control()).model_copy(deep=True)
    ctrl.ca_method = int(params.get("CA_METHOD", ctrl.ca_method))  # type: ignore[assignment]
    ctrl.reaction_torque = any(r.moment_ratio != 0.0 for r in rotors)
    curve_ct = fan_curves[curve_ref].thrust_at(1.0)
    cg = np.asarray(mass.cg_frd_m, dtype=float)
    fans: list[Fan] = []
    for i, r in enumerate(rotors):
        tilt, az = axis_to_tilt_azimuth(r.axis)
        pos = np.asarray(r.position, dtype=float) + cg
        fans.append(
            Fan(
                id=i,
                pos_frd_m=(float(pos[0]), float(pos[1]), float(pos[2])),
                tilt_deg=round(tilt, 9),
                azimuth_deg=round(az, 9),
                spin="CCW" if r.moment_ratio > 0 else "CW",
                mirror_of=None,
                curve_ref=curve_ref,
                km=abs(r.moment_ratio),
            )
        )
        if np.float32(r.thrust_coef) != np.float32(curve_ct):
            ctrl.px4_params_override[f"CA_ROTOR{i}_CT"] = float(np.float32(r.thrust_coef))
    return Scenario(
        meta=Meta(name=name, created=created, px4_version=px4_version, notes=notes),
        mass=mass,
        fans=fans,
        fan_curves=fan_curves,
        control=ctrl,
        rig=rig or Rig(),
    )


def apply_ca_params(pf: ParamFile, params: dict[str, int | float]) -> ParamFile:
    """Return a copy of pf with the given parameters replaced or appended (types from the
    existing entry, else int -> INT32 / float -> FLOAT), keeping the original order."""
    out = ParamFile(header=list(pf.header), entries=[ParamEntry(**vars(e)) for e in pf.entries])
    # keep raw text only for untouched entries (value_text() falls back to formatting otherwise)
    by_name = {e.name: e for e in out.entries}
    for name, value in params.items():
        if name in by_name:
            e = by_name[name]
            e.value = int(value) if e.type_code == PARAM_TYPE_INT32 else float(value)
        else:
            code = PARAM_TYPE_INT32 if isinstance(value, int | np.integer) else PARAM_TYPE_FLOAT
            out.entries.append(ParamEntry(1, 1, name, value, code))
    return out


def xfly80_3280_curve(estimated: bool = True) -> FanCurve:
    """Placeholder fan curve for the XFly 80 mm 3280 EDF: manufacturer maximum only (33.3 N,
    2450 W at full command, 6S). Marked estimated until thrust stand data exists."""
    return FanCurve(
        cells=6,
        points=[
            CurvePoint(cmd=0.0, thrust_N=0.0, power_W=0.0),
            CurvePoint(cmd=1.0, thrust_N=33.3, power_W=2450.0),
        ],
        lag_s=0.15,
        max_continuous_A=100.0,
        notes="manufacturer max only; replace with thrust stand data",
        estimated=estimated,
    )


def iter_rotor_param_names(count: int = NUM_FANS) -> Iterable[str]:
    for i in range(count):
        for key in ROTOR_FIELDS:
            yield f"CA_ROTOR{i}_{key}"
