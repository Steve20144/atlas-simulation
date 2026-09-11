"""Grid sweep over the geometry variable: foil deflection (when the scenario has foils) or raw
fan tilt. Finds the most efficient geometry that still controls the vehicle.

Efficiency is hover electrical power (W) from the scenario fan curves at the hover collective.
A candidate is feasible when every controlled axis is attainable in both directions, hover is
exact, headroom is at least ``min_headroom`` and yaw authority is at least ``min_yaw_Nm``.
Feasible candidates are ranked by power ascending; infeasible ones follow with their reasons.

Foil variable (``variable="foil"``): the grid values are foil deflections in degrees (0 jet
straight aft, 90 straight down, 180 straight forward). ``foil_grouping`` is ``same`` (one value
for every foil fan), ``left_right`` (independent left and right foil, grid squared) or
``per_pair`` (one value per left/right wing pair, mirrored, grid to the fourth power).

Tilt variable (``variable="tilt"``): the grid values are raw fan tilts for the four wing pairs
(rotors 0/1 outermost to 6/7 innermost, left is negative Y in FRD) with ``azimuth_mode``
``inward``, ``outward``, ``forward``, ``aft`` or the alternating fore-aft modes; ``per_pair``
gives each pair its own tilt. Centreline fans (8, 9) take ``centreline_tilts_deg``.
"""

from __future__ import annotations

import itertools
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import Any

from tiltlab.core.metrics import ControlRequirements, compute_metrics, controlled_axes
from tiltlab.scenario import Scenario

WING_PAIRS: tuple[tuple[int, int], ...] = ((0, 1), (2, 3), (4, 5), (6, 7))
CENTRELINE: tuple[int, ...] = (8, 9)
AXIS_NAMES = ("roll", "pitch", "yaw", "Fx", "Fy", "Fz")
PAIR_AZIMUTHS: dict[str, tuple[float, float]] = {
    # (left fan, right fan) azimuth in degrees; azimuth 90 tilts thrust toward +Y (right)
    "inward": (90.0, 270.0),
    "outward": (270.0, 90.0),
    "forward": (0.0, 0.0),
    "aft": (180.0, 180.0),
}
# Modes whose direction changes from pair to pair (pair index 0 = outermost): they cancel the net
# fore-aft force between pairs and give yaw from differential fore-aft thrust.
ALTERNATING_MODES: dict[str, tuple[str, ...]] = {
    "alternating": ("forward", "aft", "forward", "aft"),
    "outer_fwd_inner_aft": ("forward", "forward", "aft", "aft"),
    "outer_aft_inner_fwd": ("aft", "aft", "forward", "forward"),
}
AZIMUTH_MODES: tuple[str, ...] = tuple(PAIR_AZIMUTHS) + tuple(ALTERNATING_MODES)
VARIABLES: tuple[str, ...] = ("auto", "foil", "tilt")
FOIL_GROUPINGS: tuple[str, ...] = ("same", "left_right", "per_pair")
# reasons that only reflect a user threshold (the geometry itself can hover and steer)
THRESHOLD_REASON_PREFIXES: tuple[str, ...] = (
    "headroom", "yaw authority", "roll acceleration", "pitch acceleration",
    "yaw acceleration", "coupling", "surge leak",
)  # fmt: skip
RANK_OBJECTIVES: dict[str, str] = {
    "power": "hover power_W ascending among feasible candidates (most efficient hover)",
    "yaw": "yaw authority N m descending among feasible candidates (most yaw)",
    "yaw_per_kW": "yaw authority per kW of hover power descending (yaw bought cheapest)",
    "headroom": "hover headroom descending (most control margin)",
    "score": "composite score descending (weighted mix of headroom, power, authority, coupling)",
    "control": (
        "control score descending: weakest axis' angular acceleration over its requirement, "
        "penalised by off-axis coupling and fore-aft force leak (most authority for the pilot)"
    ),
}


def rank_key(r: dict[str, Any], rank_by: str) -> tuple[float, ...]:
    """Sort key (ascending) for one record under the chosen objective; ties break on power."""
    yaw = r["yaw_Nm"] or 0.0
    if rank_by == "control":
        return (-(r.get("control_score") or 0.0), r["power_W"])
    if rank_by == "yaw":
        return (-yaw, r["power_W"])
    if rank_by == "yaw_per_kW":
        return (-(r["yaw_Nm_per_kW"] or 0.0), r["power_W"])
    if rank_by == "headroom":
        return (-r["headroom"], r["power_W"])
    if rank_by == "score":
        return (-r["score"], r["power_W"])
    return (r["power_W"], -yaw)


def pair_azimuths(mode: str, pair_index: int) -> tuple[float, float]:
    """(left, right) azimuth in degrees for wing pair ``pair_index`` (0 is outermost)."""
    if mode in ALTERNATING_MODES:
        return PAIR_AZIMUTHS[ALTERNATING_MODES[mode][pair_index]]
    return PAIR_AZIMUTHS[mode]


@dataclass
class SweepSpec:
    tilts_deg: list[float]  # grid values: foil deflections (0..180) or fan tilts (0..90)
    variable: str = "auto"
    foil_grouping: str = "same"
    azimuth_mode: str = "forward"
    per_pair: bool = False
    centreline_tilts_deg: list[float] = field(default_factory=lambda: [0.0])
    centreline_azimuth_deg: float = 0.0
    concept: str = "stock"
    collective: float | None = None
    min_headroom: float = 0.2
    min_yaw_Nm: float = 0.0
    # control checks (metrics.ControlRequirements): 0 / 1 means "do not filter on this"
    min_roll_accel: float = 0.0  # rad/s^2 from attainable roll torque over inertia
    min_pitch_accel: float = 0.0
    min_yaw_accel: float = 0.0
    max_coupling: float = 1.0  # off-axis leakage fraction through the PX4 allocator
    max_surge_leak: float = 1.0  # fore-aft/lateral force leak, fraction of weight
    # hover attitudes to try (nose-up degrees); every geometry is evaluated at each one
    hover_pitch_deg: list[float] = field(default_factory=lambda: [0.0])
    rank_by: str = "power"  # power | yaw | yaw_per_kW | headroom | score | control
    max_candidates: int = 5000

    def __post_init__(self) -> None:
        for name in ("min_roll_accel", "min_pitch_accel", "min_yaw_accel"):
            if float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} must be >= 0")
        for p in self.hover_pitch_deg:
            if not -90.0 <= float(p) <= 90.0:
                raise ValueError("hover_pitch_deg must lie in [-90, 90] degrees")
        for name in ("max_coupling", "max_surge_leak"):
            if not 0.0 <= float(getattr(self, name)) <= 1.0:
                raise ValueError(f"{name} must lie in [0, 1]")
        if self.variable not in VARIABLES:
            raise ValueError(f"variable must be one of {list(VARIABLES)}")
        if self.foil_grouping not in FOIL_GROUPINGS:
            raise ValueError(f"foil_grouping must be one of {list(FOIL_GROUPINGS)}")
        if self.azimuth_mode not in AZIMUTH_MODES:
            raise ValueError(f"azimuth_mode must be one of {list(AZIMUTH_MODES)}")
        if self.rank_by not in RANK_OBJECTIVES:
            raise ValueError(f"rank_by must be one of {list(RANK_OBJECTIVES)}")
        if not self.tilts_deg:
            raise ValueError("tilts_deg is empty")
        for t in list(self.tilts_deg) + list(self.centreline_tilts_deg):
            if not 0.0 <= float(t) <= 180.0:
                raise ValueError("angles must lie in [0, 180] degrees")

    def resolve_variable(self, scenario: Scenario) -> str:
        if self.variable == "auto":
            return "foil" if scenario.foils else "tilt"
        return self.variable


def _left_right(scenario: Scenario, pair: tuple[int, int]) -> tuple[int, int]:
    """Rotor ids of a wing pair ordered (left, right) by their FRD Y position."""
    a, b = pair
    fa = next(f for f in scenario.fans if f.id == a)
    fb = next(f for f in scenario.fans if f.id == b)
    return (a, b) if fa.pos_frd_m[1] <= fb.pos_frd_m[1] else (b, a)


# ---------------------------------------------------------------- tilt variable


def candidate_angles(
    scenario: Scenario, spec: SweepSpec
) -> Iterator[dict[int, tuple[float, float]]]:
    """Yield {rotor id: (tilt_deg, azimuth_deg)} for every grid point of the tilt variable."""
    for t in list(spec.tilts_deg) + list(spec.centreline_tilts_deg):
        if float(t) > 90.0:
            raise ValueError("fan tilt angles must lie in [0, 90] degrees")
    pair_lr = [_left_right(scenario, p) for p in WING_PAIRS]
    if spec.per_pair:
        pair_grids: Iterator[tuple[float, ...]] = itertools.product(
            spec.tilts_deg, repeat=len(WING_PAIRS)
        )
    else:
        pair_grids = ((t,) * len(WING_PAIRS) for t in spec.tilts_deg)
    for pair_tilts in pair_grids:
        for ct in spec.centreline_tilts_deg:
            angles: dict[int, tuple[float, float]] = {}
            for pi, ((left, right), tilt) in enumerate(zip(pair_lr, pair_tilts, strict=True)):
                az_left, az_right = pair_azimuths(spec.azimuth_mode, pi)
                angles[left] = (float(tilt), az_left)
                angles[right] = (float(tilt), az_right)
            for rid in CENTRELINE:
                angles[rid] = (float(ct), float(spec.centreline_azimuth_deg))
            yield angles


def apply_angles(scenario: Scenario, angles: dict[int, tuple[float, float]]) -> Scenario:
    fans = [
        f.model_copy(update={"tilt_deg": angles[f.id][0], "azimuth_deg": angles[f.id][1]})
        if f.id in angles
        else f
        for f in scenario.fans
    ]
    return scenario.model_copy(update={"fans": fans})


# ---------------------------------------------------------------- foil variable


def foil_fan_ids(scenario: Scenario) -> list[int]:
    return sorted(fid for foil in scenario.foils for fid in foil.fan_ids)


def candidate_deflections(scenario: Scenario, spec: SweepSpec) -> Iterator[dict[int, float]]:
    """Yield {fan id: deflection_deg} for every grid point of the foil variable."""
    ids = foil_fan_ids(scenario)
    if not ids:
        raise ValueError("scenario has no foils")
    fans = {f.id: f for f in scenario.fans}
    left = [i for i in ids if fans[i].pos_frd_m[1] < 0]
    right = [i for i in ids if fans[i].pos_frd_m[1] >= 0]
    if spec.foil_grouping == "same":
        for d in spec.tilts_deg:
            yield {i: float(d) for i in ids}
    elif spec.foil_grouping == "left_right":
        for dl, dr in itertools.product(spec.tilts_deg, repeat=2):
            out = {i: float(dl) for i in left}
            out.update({i: float(dr) for i in right})
            yield out
    else:  # per_pair: mirrored left/right, one value per wing pair that has foil fans
        pairs = [p for p in WING_PAIRS if p[0] in ids and p[1] in ids]
        loose = [i for i in ids if not any(i in p for p in pairs)]
        for values in itertools.product(spec.tilts_deg, repeat=len(pairs)):
            out: dict[int, float] = {}
            for (a, b), d in zip(pairs, values, strict=True):
                out[a] = float(d)
                out[b] = float(d)
            for i in loose:
                out[i] = float(values[0]) if values else float(spec.tilts_deg[0])
            yield out


def apply_deflections(scenario: Scenario, deflections: dict[int, float]) -> Scenario:
    """Set per-fan foil deflections (collapsing to the foil's deflection_deg when uniform)."""
    foils = []
    for foil in scenario.foils:
        own = {i: deflections[i] for i in foil.fan_ids if i in deflections}
        if not own:
            foils.append(foil)
            continue
        values = set(own.values())
        if len(values) == 1 and len(own) == len(foil.fan_ids):
            foils.append(
                foil.model_copy(
                    update={"deflection_deg": values.pop(), "per_fan_deflection_deg": {}}
                )
            )
        else:
            merged = dict(foil.per_fan_deflection_deg)
            merged.update(own)
            foils.append(foil.model_copy(update={"per_fan_deflection_deg": merged}))
    return scenario.model_copy(update={"foils": foils})


# ---------------------------------------------------------------- evaluation


def _min_authority(auth: dict[str, Any], axis: str) -> float | None:
    a = auth.get(axis)
    if not a or not (a.get("plus_attainable") and a.get("minus_attainable")):
        return None
    return float(min(abs(a["plus"]), abs(a["minus"])))


def evaluate_scenario(sc: Scenario, spec: SweepSpec) -> dict[str, Any]:
    """Metrics of one geometry reduced to the sweep record (geometry fields added by caller)."""
    defaults = ControlRequirements()
    req = ControlRequirements(
        min_roll_accel=spec.min_roll_accel or defaults.min_roll_accel,
        min_pitch_accel=spec.min_pitch_accel or defaults.min_pitch_accel,
        min_yaw_accel=spec.min_yaw_accel or defaults.min_yaw_accel,
        max_coupling=spec.max_coupling if spec.max_coupling < 1.0 else defaults.max_coupling,
        max_surge_leak=(
            spec.max_surge_leak if spec.max_surge_leak < 1.0 else defaults.max_surge_leak
        ),
    )
    m = compute_metrics(sc, spec.concept, spec.collective, requirements=req)
    hover = m["hover"]
    auth = m["authority"]
    ctrl = m["control"]
    reasons: list[str] = []
    for k in controlled_axes(spec.concept):
        if _min_authority(auth, AXIS_NAMES[k]) is None:
            reasons.append(f"{AXIS_NAMES[k]} unattainable")
    cond = m["conditioning"].get("condition_number")
    separated = [f.id for f in sc.fans_sorted() if not sc.fan_jet_attached(f)]
    if separated:
        foil = sc.foil_for(separated[0])
        limit = foil.coanda.separation_deg() if foil and foil.coanda else 0.0
        reasons.append(
            f"jet separates from the Coanda surface on motors {separated} "
            f"(attachment limit {limit:.0f} deg)"
        )
    if float(m.get("collective_hover", 0.0)) > 1.0:
        reasons.append(
            "cannot lift the aircraft: vertical thrust at full command is below the weight"
        )
    elif hover.get("exact") is False:
        collapsed = max(hover["u"]) <= 1e-6
        fz_ok = _min_authority(auth, "Fz") is not None
        if collapsed and not fz_ok:
            reasons.append(
                "no level-attitude hover trim: the redirected jets' net Fx or Fy cannot be zeroed"
            )
        else:
            cond_txt = f" (B condition number {cond:.0f})" if isinstance(cond, int | float) else ""
            reasons.append(
                ("PX4 allocator collapsed to zero thrust" if collapsed else "hover trim not exact")
                + cond_txt
            )
    if float(hover["headroom"]) < spec.min_headroom:
        reasons.append(f"headroom {float(hover['headroom']):.2f} below {spec.min_headroom:.2f}")
    yaw = _min_authority(auth, "yaw")
    if yaw is not None and yaw < spec.min_yaw_Nm:
        reasons.append(f"yaw authority {yaw:.2f} N m below {spec.min_yaw_Nm:.2f}")
    # control checks against the sweep's own thresholds (0 / 1 = not filtered)
    acc = {
        a: float(ctrl["axes"][a]["accel_rad_s2"]) if a in ctrl["axes"] else None
        for a in AXIS_NAMES[:3]
    }
    for axis, limit in (
        ("roll", spec.min_roll_accel),
        ("pitch", spec.min_pitch_accel),
        ("yaw", spec.min_yaw_accel),
    ):
        if limit > 0.0 and acc[axis] is not None and acc[axis] < limit:
            reasons.append(f"{axis} acceleration {acc[axis]:.1f} rad/s2 below {limit:.1f}")
    coupling_max = m["coupling"].get("max_offaxis_fraction")
    if spec.max_coupling < 1.0 and coupling_max is not None and coupling_max > spec.max_coupling:
        reasons.append(f"coupling {coupling_max:.2f} above {spec.max_coupling:.2f}")
    surge = max((float(a["surge_leak_frac_of_weight"]) for a in ctrl["axes"].values()), default=0.0)
    if spec.max_surge_leak < 1.0 and surge > spec.max_surge_leak:
        reasons.append(f"surge leak {surge:.2f} of weight above {spec.max_surge_leak:.2f}")
    linear = min(
        (float(a["linear_fraction"]) for a in ctrl["axes"].values() if a["attainable"]),
        default=0.0,
    )
    power = float(hover["power_W"])
    return {
        "power_W": power,
        "headroom": float(hover["headroom"]),
        "roll_Nm": _min_authority(auth, "roll"),
        "pitch_Nm": _min_authority(auth, "pitch"),
        "yaw_Nm": yaw,
        "fz_up_N": _min_authority(auth, "Fz"),
        "yaw_Nm_per_kW": (yaw / (power / 1000.0)) if (yaw is not None and power > 1e-9) else None,
        "coupling_max": coupling_max,
        "condition_number": float(cond) if isinstance(cond, int | float) else None,
        "roll_acc": acc["roll"],
        "pitch_acc": acc["pitch"],
        "yaw_acc": acc["yaw"],
        "linear_frac": linear,
        "surge_leak": surge,
        "control_score": float(ctrl["score"]),
        "weakest_axis": ctrl["weakest_axis"],
        "score": float(m["score"]["value"]),
        "estimated": bool(m.get("estimated", False)),
        "feasible": not reasons,
        "reasons": reasons,
    }


def evaluate_candidate(
    scenario: Scenario, angles: dict[int, tuple[float, float]], spec: SweepSpec
) -> dict[str, Any]:
    """Tilt-variable candidate: fan tilt/azimuth per rotor."""
    sc = apply_angles(scenario, angles)
    ordered = sorted(angles)
    rec = {
        "variable": "tilt",
        "tilts_deg": [angles[i][0] for i in ordered],
        "azimuths_deg": [angles[i][1] for i in ordered],
        "pair_tilts_deg": [angles[_left_right(scenario, p)[0]][0] for p in WING_PAIRS],
        "centreline_tilt_deg": angles[CENTRELINE[0]][0],
    }
    rec.update(evaluate_scenario(sc, spec))
    return rec


def evaluate_foil_candidate(
    scenario: Scenario, deflections: dict[int, float], spec: SweepSpec
) -> dict[str, Any]:
    """Foil-variable candidate: deflection per foil fan; pair_tilts_deg carries the per-pair
    deflection so tables and the UI can show it in the same column."""
    sc = apply_deflections(scenario, deflections)
    fans = {f.id: f for f in scenario.fans}
    pairs = []
    for a, b in WING_PAIRS:
        if a in deflections or b in deflections:
            left, right = _left_right(scenario, (a, b))
            pairs.append(deflections.get(left, deflections.get(right, 0.0)))
    left_vals = [d for i, d in deflections.items() if fans[i].pos_frd_m[1] < 0]
    right_vals = [d for i, d in deflections.items() if fans[i].pos_frd_m[1] >= 0]
    rec = {
        "variable": "foil",
        "deflections_deg": {str(i): deflections[i] for i in sorted(deflections)},
        "pair_tilts_deg": pairs,
        "left_deg": (sum(left_vals) / len(left_vals)) if left_vals else None,
        "right_deg": (sum(right_vals) / len(right_vals)) if right_vals else None,
        "centreline_tilt_deg": float(next((f.tilt_deg for f in scenario.fans if f.id == 8), 0.0)),
    }
    rec.update(evaluate_scenario(sc, spec))
    return rec


def _with_hover_pitch(scenario: Scenario, pitch_deg: float) -> Scenario:
    """Copy of the scenario hovering at pitch_deg nose-up (frame.hover_pitch_deg)."""
    sc = scenario.model_copy(deep=True)
    sc.frame.hover_pitch_deg = pitch_deg
    return sc


def run_sweep(scenario: Scenario, spec: SweepSpec) -> dict[str, Any]:
    """Evaluate the whole grid and rank: feasible first, then hover power ascending, then yaw
    descending."""
    t0 = time.perf_counter()
    variable = spec.resolve_variable(scenario)
    records: list[dict[str, Any]] = []
    variants = [(float(p), _with_hover_pitch(scenario, float(p))) for p in spec.hover_pitch_deg]
    truncated = False
    if variable == "foil":
        for n, defl in enumerate(candidate_deflections(scenario, spec)):
            if n >= spec.max_candidates:
                truncated = True
                break
            for pitch, sc0 in variants:
                rec = evaluate_foil_candidate(sc0, defl, spec)
                rec["hover_pitch_deg"] = pitch
                records.append(rec)
    else:
        for n, angles in enumerate(candidate_angles(scenario, spec)):
            if n >= spec.max_candidates:
                truncated = True
                break
            for pitch, sc0 in variants:
                rec = evaluate_candidate(sc0, angles, spec)
                rec["hover_pitch_deg"] = pitch
                records.append(rec)
    records.sort(key=lambda r: (not r["feasible"], *rank_key(r, spec.rank_by)))
    feasible = [r for r in records if r["feasible"]]
    return {
        "diagnostics": sweep_diagnostics(records),
        "variable": variable,
        "n_evaluated": len(records),
        "n_feasible": len(feasible),
        "truncated": truncated,
        "elapsed_ms": (time.perf_counter() - t0) * 1e3,
        "objective": RANK_OBJECTIVES[spec.rank_by],
        "spec": {
            "variable": variable,
            "tilts_deg": list(spec.tilts_deg),
            "foil_grouping": spec.foil_grouping,
            "azimuth_mode": spec.azimuth_mode,
            "per_pair": spec.per_pair,
            "centreline_tilts_deg": list(spec.centreline_tilts_deg),
            "concept": spec.concept,
            "collective": spec.collective,
            "min_headroom": spec.min_headroom,
            "min_yaw_Nm": spec.min_yaw_Nm,
            "min_roll_accel": spec.min_roll_accel,
            "min_pitch_accel": spec.min_pitch_accel,
            "min_yaw_accel": spec.min_yaw_accel,
            "max_coupling": spec.max_coupling,
            "max_surge_leak": spec.max_surge_leak,
            "hover_pitch_deg": spec.hover_pitch_deg,
            "rank_by": spec.rank_by,
        },
        "candidates": records,
        "best": feasible[0] if feasible else None,
    }


def _threshold_only(reasons: list[str]) -> bool:
    """True when a candidate failed only the user thresholds (headroom, minimum yaw), i.e. it can
    hover level and steer every axis."""
    return bool(reasons) and all(r.startswith(THRESHOLD_REASON_PREFIXES) for r in reasons)


def sweep_diagnostics(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Why a grid produced few or no feasible rows: how many geometries are controllable at all,
    the best headroom and yaw among them, and the most common reason among the near misses."""
    controllable = [r for r in records if r["feasible"] or _threshold_only(r["reasons"])]
    blocked = [r for r in controllable if not r["feasible"]]
    counts: dict[str, int] = {}
    near = blocked if blocked else [r for r in records if not r["feasible"]]
    if near:
        fewest = min(len(r["reasons"]) for r in near)
        for r in near:
            if len(r["reasons"]) == fewest:
                for reason in r["reasons"]:
                    key = reason.split(" (")[0]
                    counts[key] = counts.get(key, 0) + 1
    most_common = max(counts.items(), key=lambda kv: kv[1])[0] if counts else None
    return {
        "n_controllable": len(controllable),
        "n_blocked_by_thresholds": len(blocked),
        "best_headroom_controllable": (
            max(r["headroom"] for r in controllable) if controllable else None
        ),
        "best_yaw_controllable": (
            max((r["yaw_Nm"] or 0.0) for r in controllable) if controllable else None
        ),
        "most_common_reason_near_miss": most_common,
    }


def sweep_table(result: dict[str, Any], top: int = 15) -> str:
    """Compact text table of the ranked candidates."""
    label = "foil defl deg" if result.get("variable") == "foil" else "pair tilts deg"
    head = (
        label, "ctr", "power W", "headrm", "roll", "pitch", "yaw", "r a/s2", "p a/s2", "y a/s2",
        "coupl", "surge", "ctrl", "score",
    )  # fmt: skip
    rows = [
        f"{head[0]:>22s} {head[1]:>4s} {head[2]:>8s} {head[3]:>6s} {head[4]:>6s} {head[5]:>6s} "
        f"{head[6]:>6s} {head[7]:>6s} {head[8]:>6s} {head[9]:>6s} {head[10]:>6s} {head[11]:>6s} "
        f"{head[12]:>6s} {head[13]:>6s}  feasible"
    ]

    def f(v: float | None, fmt: str) -> str:
        return format(v, fmt) if v is not None else "-"

    for r in result["candidates"][:top]:
        tilts = str([round(t, 1) for t in r["pair_tilts_deg"]])
        if r.get("hover_pitch_deg"):
            tilts += f" @{r['hover_pitch_deg']:+.0f}"
        verdict = "yes" if r["feasible"] else "; ".join(r["reasons"])
        rows.append(
            f"{tilts:>22s} {r['centreline_tilt_deg']:4.0f} {r['power_W']:8.0f} "
            f"{r['headroom']:6.2f} {f(r['roll_Nm'], '6.2f')} {f(r['pitch_Nm'], '6.2f')} "
            f"{f(r['yaw_Nm'], '6.2f')} {f(r.get('roll_acc'), '6.1f')} "
            f"{f(r.get('pitch_acc'), '6.1f')} {f(r.get('yaw_acc'), '6.1f')} "
            f"{f(r['coupling_max'], '6.3f')} {f(r.get('surge_leak'), '6.3f')} "
            f"{f(r.get('control_score'), '6.2f')} {r['score']:6.3f}  {verdict}"
        )
    return "\n".join(rows)
