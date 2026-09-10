"""Grid sweep over fan tilt angles: find the most efficient geometry that still controls the
vehicle.

Efficiency is hover electrical power (W) from the scenario fan curves at the hover collective. A
candidate
is feasible when every controlled axis is attainable in both directions, hover is exact, headroom
is at
least ``min_headroom`` and yaw authority is at least ``min_yaw_Nm``. Feasible candidates are ranked
by
power ascending; infeasible ones follow, ranked the same way, with their reasons listed.

Angles: the eight wing fans form four left/right pairs (rotors 0/1 outermost to 6/7 innermost, left
is
negative Y in FRD). ``azimuth_mode`` sets how a pair tilts: ``inward`` (left fan thrust toward +Y,
right
toward -Y), ``outward``, ``forward`` or ``aft``. With ``per_pair`` each pair gets its own tilt from
the
grid (len(tilts)**4 candidates); otherwise one shared tilt. The centreline fans (8, 9) take
``centreline_tilts_deg`` with ``centreline_azimuth_deg``.
"""

from __future__ import annotations

import itertools
import time
from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import Any

from tiltlab.core.metrics import compute_metrics, controlled_axes
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


@dataclass
class SweepSpec:
    tilts_deg: list[float]
    azimuth_mode: str = "inward"
    per_pair: bool = False
    centreline_tilts_deg: list[float] = field(default_factory=lambda: [0.0])
    centreline_azimuth_deg: float = 0.0
    concept: str = "stock"
    collective: float | None = None
    min_headroom: float = 0.2
    min_yaw_Nm: float = 0.0
    max_candidates: int = 5000

    def __post_init__(self) -> None:
        if self.azimuth_mode not in PAIR_AZIMUTHS:
            raise ValueError(f"azimuth_mode must be one of {sorted(PAIR_AZIMUTHS)}")
        if not self.tilts_deg:
            raise ValueError("tilts_deg is empty")
        for t in list(self.tilts_deg) + list(self.centreline_tilts_deg):
            if not 0.0 <= float(t) <= 90.0:
                raise ValueError("tilt angles must lie in [0, 90] degrees")


def _left_right(scenario: Scenario, pair: tuple[int, int]) -> tuple[int, int]:
    """Rotor ids of a wing pair ordered (left, right) by their FRD Y position."""
    a, b = pair
    fa = next(f for f in scenario.fans if f.id == a)
    fb = next(f for f in scenario.fans if f.id == b)
    return (a, b) if fa.pos_frd_m[1] <= fb.pos_frd_m[1] else (b, a)


def candidate_angles(
    scenario: Scenario, spec: SweepSpec
) -> Iterator[dict[int, tuple[float, float]]]:
    """Yield {rotor id: (tilt_deg, azimuth_deg)} for every grid point."""
    az_left, az_right = PAIR_AZIMUTHS[spec.azimuth_mode]
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
            for (left, right), tilt in zip(pair_lr, pair_tilts, strict=True):
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


def _min_authority(auth: dict[str, Any], axis: str) -> float | None:
    a = auth.get(axis)
    if not a or not (a.get("plus_attainable") and a.get("minus_attainable")):
        return None
    return float(min(abs(a["plus"]), abs(a["minus"])))


def evaluate_candidate(
    scenario: Scenario, angles: dict[int, tuple[float, float]], spec: SweepSpec
) -> dict[str, Any]:
    """Metrics of one geometry reduced to the sweep record."""
    sc = apply_angles(scenario, angles)
    m = compute_metrics(sc, spec.concept, spec.collective)
    hover = m["hover"]
    auth = m["authority"]
    reasons: list[str] = []
    for k in controlled_axes(spec.concept):
        if _min_authority(auth, AXIS_NAMES[k]) is None:
            reasons.append(f"{AXIS_NAMES[k]} unattainable")
    cond = m["conditioning"].get("condition_number")
    if hover.get("exact") is False:
        collapsed = max(hover["u"]) <= 1e-6
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
    power = float(hover["power_W"])
    ordered = sorted(angles)
    return {
        "tilts_deg": [angles[i][0] for i in ordered],
        "azimuths_deg": [angles[i][1] for i in ordered],
        "pair_tilts_deg": [angles[_left_right(scenario, p)[0]][0] for p in WING_PAIRS],
        "centreline_tilt_deg": angles[CENTRELINE[0]][0],
        "power_W": power,
        "headroom": float(hover["headroom"]),
        "roll_Nm": _min_authority(auth, "roll"),
        "pitch_Nm": _min_authority(auth, "pitch"),
        "yaw_Nm": yaw,
        "fz_up_N": _min_authority(auth, "Fz"),
        "yaw_Nm_per_kW": (yaw / (power / 1000.0)) if (yaw is not None and power > 1e-9) else None,
        "coupling_max": m["coupling"].get("max_offaxis_fraction"),
        "condition_number": float(cond) if isinstance(cond, int | float) else None,
        "score": float(m["score"]["value"]),
        "estimated": bool(m.get("estimated", False)),
        "feasible": not reasons,
        "reasons": reasons,
    }


def run_sweep(scenario: Scenario, spec: SweepSpec) -> dict[str, Any]:
    """Evaluate the whole grid and rank: feasible first, then hover power ascending, then yaw
    descending."""
    t0 = time.perf_counter()
    records: list[dict[str, Any]] = []
    truncated = False
    for n, angles in enumerate(candidate_angles(scenario, spec)):
        if n >= spec.max_candidates:
            truncated = True
            break
        records.append(evaluate_candidate(scenario, angles, spec))
    records.sort(key=lambda r: (not r["feasible"], r["power_W"], -(r["yaw_Nm"] or 0.0)))
    feasible = [r for r in records if r["feasible"]]
    return {
        "n_evaluated": len(records),
        "n_feasible": len(feasible),
        "truncated": truncated,
        "elapsed_ms": (time.perf_counter() - t0) * 1e3,
        "objective": "hover power_W ascending among feasible candidates",
        "spec": {
            "tilts_deg": list(spec.tilts_deg),
            "azimuth_mode": spec.azimuth_mode,
            "per_pair": spec.per_pair,
            "centreline_tilts_deg": list(spec.centreline_tilts_deg),
            "concept": spec.concept,
            "collective": spec.collective,
            "min_headroom": spec.min_headroom,
            "min_yaw_Nm": spec.min_yaw_Nm,
        },
        "candidates": records,
        "best": feasible[0] if feasible else None,
    }


def sweep_table(result: dict[str, Any], top: int = 15) -> str:
    """Compact text table of the ranked candidates."""
    head = (
        "pair tilts deg",
        "ctr",
        "power W",
        "headrm",
        "roll",
        "pitch",
        "yaw",
        "yaw/kW",
        "coupl",
        "score",
    )
    rows = [
        f"{head[0]:>22s} {head[1]:>4s} {head[2]:>8s} {head[3]:>6s} {head[4]:>6s} {head[5]:>6s} "
        f"{head[6]:>6s} {head[7]:>7s} {head[8]:>6s} {head[9]:>6s}  feasible"
    ]

    def f(v: float | None, fmt: str) -> str:
        return format(v, fmt) if v is not None else "-"

    for r in result["candidates"][:top]:
        tilts = str([round(t, 1) for t in r["pair_tilts_deg"]])
        verdict = "yes" if r["feasible"] else "; ".join(r["reasons"])
        rows.append(
            f"{tilts:>22s} {r['centreline_tilt_deg']:4.0f} {r['power_W']:8.0f} "
            f"{r['headroom']:6.2f} {f(r['roll_Nm'], '6.2f')} {f(r['pitch_Nm'], '6.2f')} "
            f"{f(r['yaw_Nm'], '6.2f')} {f(r['yaw_Nm_per_kW'], '7.3f')} "
            f"{f(r['coupling_max'], '6.3f')} {r['score']:6.3f}  {verdict}"
        )
    return "\n".join(rows)
