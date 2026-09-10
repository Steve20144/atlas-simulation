"""Static analysis metrics for one Scenario and control concept (PLAN.md section 6, M4).

Everything is evaluated at a stated collective (fraction of the vertical thrust available at
u = 1 on every fan; default the hover collective) around the trim that PX4 would fly: torque
setpoint zero, Fx = Fy = 0 as rows of the effectiveness matrix (the stock allocator always
solves all six rows) and Fz = -collective * Fz_max.

Frames and units: the effectiveness matrix B (6 x N) is the PX4 matrix of geometry.py, rows
[roll, pitch, yaw] in N m and [Fx, Fy, Fz] in N, FRD body frame, per unit normalised command
u in [0, 1]. CT comes from the scenario fan curve at cmd 1.0 (N), KM from the reaction-torque
toggle (dimensionless); CA_ROTORn_CT overrides in the scenario are ignored here because they
describe what the flight controller was told, not what the fan produces. Power is electrical
input power in W from the fan curve. The PX4 normalised control units of allocation.py relate
to physical units through the allocator scale: c_norm = physical * scale[axis].
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np
from scipy.optimize import linprog, lsq_linear

from tiltlab.core.allocation import ControlAllocatorReplica
from tiltlab.core.fan import FanCurve
from tiltlab.core.geometry import Rotor, compute_effectiveness_matrix, rotors_from_scenario
from tiltlab.scenario import Scenario

AXIS_NAMES: tuple[str, ...] = ("roll", "pitch", "yaw", "Fx", "Fy", "Fz")
AXIS_UNITS: tuple[str, ...] = ("N m", "N m", "N m", "N", "N", "N")
TORQUE_AXES: tuple[int, ...] = (0, 1, 2)
STOCK_AXES: tuple[int, ...] = (0, 1, 2, 5)
ALL_AXES: tuple[int, ...] = (0, 1, 2, 3, 4, 5)
CONCEPTS: tuple[str, ...] = ("stock", "fully_actuated")

BADGE_STOCK = "stock_px4"
BADGE_FA = "needs_fully_actuated_controller"

COUPLING_FRACTION = 0.2  # PLAN.md M4: apply 20 percent of each axis authority
DEFAULT_WEIGHTS: dict[str, float] = {
    "hover_headroom": 1.0,
    "power_margin": 1.0,
    "authority_balance": 1.0,
    "decoupling": 1.0,
    "conditioning": 1.0,
}
_EPS = 1e-9


def _py(obj: Any) -> Any:
    """Recursively convert numpy scalars and arrays to JSON-serialisable Python objects."""
    if isinstance(obj, dict):
        return {str(k): _py(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_py(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return [_py(v) for v in obj.tolist()]
    if isinstance(obj, (np.floating, float)):
        f = float(obj)
        return f if np.isfinite(f) else None
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    return obj


class _FanBank:
    """Vectorised thrust (N) and electrical power (W) of all fans from their curves."""

    def __init__(self, scenario: Scenario) -> None:
        fans = scenario.fans_sorted()
        self.refs = [f.curve_ref for f in fans]
        self.curves: dict[str, FanCurve] = {
            ref: FanCurve.from_scenario_curve(scenario.fan_curves[ref].model_dump())
            for ref in set(self.refs)
        }
        self.groups = {
            ref: np.array([i for i, r in enumerate(self.refs) if r == ref]) for ref in self.curves
        }
        self.ct = np.array([self.curves[r].ct_for_px4() for r in self.refs])
        self.estimated = any(c.estimated for c in self.curves.values())

    def thrust(self, u: np.ndarray) -> np.ndarray:
        """Per-fan thrust (N) at commands u (N,), curve clamped to [0, 1]."""
        out = np.empty_like(u, dtype=float)
        for ref, idx in self.groups.items():
            out[idx] = self.curves[ref].thrust(u[idx])
        return out

    def power(self, u: np.ndarray) -> np.ndarray:
        """Per-fan electrical power (W) at commands u (N,), curve clamped to [0, 1]."""
        out = np.empty_like(u, dtype=float)
        for ref, idx in self.groups.items():
            out[idx] = self.curves[ref].power(u[idx])
        return out

    def total_power(self, u: np.ndarray) -> float:
        return float(self.power(u).sum())

    def max_power(self) -> float:
        return float(sum(self.curves[r].max_power_w for r in self.refs))


def build_effectiveness(scenario: Scenario) -> tuple[np.ndarray, _FanBank]:
    """PX4 effectiveness matrix (6 x N, N m and N per unit command, FRD) of the scenario with
    CT from the fan curve at cmd 1.0 and KM from the reaction-torque toggle, plus the fan bank."""
    bank = _FanBank(scenario)
    rotors = [
        Rotor(r.position, r.axis, float(ct), r.moment_ratio)
        for r, ct in zip(rotors_from_scenario(scenario), bank.ct, strict=True)
    ]
    full, n = compute_effectiveness_matrix(rotors)
    return full[:, :n], bank


def controlled_axes(concept: str) -> tuple[int, ...]:
    if concept == "stock":
        return STOCK_AXES
    if concept == "fully_actuated":
        return ALL_AXES
    raise ValueError(f"unknown concept '{concept}', expected one of {CONCEPTS}")


def _axis_authority(
    b: np.ndarray, base: np.ndarray, axis: int, sign: int
) -> tuple[float | None, np.ndarray | None]:
    """Largest sign * delta on one axis (N m or N) with B u = base + sign * delta * e_axis and
    0 <= u <= 1 (scipy linprog, HiGHS). Returns (delta, u) or (None, None) if unattainable."""
    n = b.shape[1]
    e = np.zeros((6, 1))
    e[axis, 0] = -float(sign)
    a_eq = np.hstack([b, e])
    cost = np.zeros(n + 1)
    cost[-1] = -1.0
    res = linprog(
        cost, A_eq=a_eq, b_eq=base, bounds=[(0.0, 1.0)] * n + [(0.0, None)], method="highs"
    )
    if res.status != 0 or res.x is None:
        return None, None
    return max(0.0, float(res.x[-1])), np.clip(res.x[:n], 0.0, 1.0)


def _hover_bounded_lsq(b: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, bool]:
    """Min-norm solution of B u = target with 0 <= u <= 1: the pseudo-inverse solution when it
    is inside the bounds, otherwise a bounded least squares fit that weights the equality
    residual 1e3 times more than the command norm. Returns (u, exact)."""
    u = np.linalg.pinv(b) @ target
    if np.all(u >= -1e-9) and np.all(u <= 1.0 + 1e-9):
        return np.clip(u, 0.0, 1.0), True
    w = 1e3
    a = np.vstack([b * w, np.eye(b.shape[1])])
    rhs = np.concatenate([target * w, np.zeros(b.shape[1])])
    res = lsq_linear(a, rhs, bounds=(0.0, 1.0), method="bvls")
    u = np.clip(res.x, 0.0, 1.0)
    exact = bool(np.linalg.norm(b @ u - target) < 1e-3 * max(1.0, np.linalg.norm(target)))
    return u, exact


def _normalise_weights(weights: dict[str, float] | None) -> dict[str, float]:
    merged = dict(DEFAULT_WEIGHTS)
    if weights:
        unknown = set(weights) - set(DEFAULT_WEIGHTS)
        if unknown:
            raise ValueError(
                f"unknown score weights {sorted(unknown)}; allowed {sorted(DEFAULT_WEIGHTS)}"
            )
        for k, v in weights.items():
            if not np.isfinite(v) or v < 0.0:
                raise ValueError(f"weight '{k}' must be a finite number >= 0")
            merged[k] = float(v)
    return merged


def compute_metrics(
    scenario: Scenario,
    concept: str = "stock",
    collective: float | None = None,
    weights: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Static metrics of a scenario under a control concept, at a stated collective.

    concept "stock": PX4 as flown, controlled axes roll, pitch, yaw, Fz with Fx = Fy = 0 kept
    as matrix rows; hover through the PX4 allocator replica (CA_METHOD of the scenario).
    "fully_actuated": all six axes; hover by bounded min-norm least squares, allocator with
    CA_METHOD 0. collective is the fraction of Fz_max (sum of vertical thrust at u = 1) that
    is commanded (default: hover, weight / Fz_max). weights: composite score weights, keys of
    DEFAULT_WEIGHTS. Returns a JSON-serialisable dict; units are stated in the keys.
    """
    t_start = time.perf_counter()
    axes = controlled_axes(concept)
    w = _normalise_weights(weights)
    b, bank = build_effectiveness(scenario)
    n = b.shape[1]
    notes: list[str] = []

    overrides = [k for k in scenario.control.px4_params_override if k.endswith("_CT")]
    if overrides:
        notes.append(
            f"{len(overrides)} CA_ROTORn_CT override(s) ignored: metrics use the fan curve CT "
            f"{np.round(bank.ct, 3).tolist()} N"
        )

    gravity = float(scenario.environment.gravity)
    weight_n = float(scenario.mass.total_kg) * gravity
    fz_max = float(-b[5].sum())  # N, all fans at u = 1 (upward is -Z in FRD)
    if fz_max <= _EPS:
        raise ValueError("scenario produces no upward thrust")
    hover_collective = weight_n / fz_max
    coll = hover_collective if collective is None else float(collective)
    if not 0.0 <= coll <= 1.0:
        raise ValueError("collective must lie in [0, 1]")
    fz_target = -coll * fz_max
    base = np.zeros(6)
    base[5] = fz_target

    # --- allocator replica (PX4 path) ------------------------------------------------------
    ca_method = int(scenario.control.ca_method) if concept == "stock" else 0
    rep = ControlAllocatorReplica(b, ca_method=ca_method, mc_airmode=0)
    scale = np.asarray(rep.scale, dtype=float)  # normalised per physical unit, per axis

    def to_norm(phys: np.ndarray) -> np.ndarray:
        return np.where(scale > _EPS, phys * scale, 0.0)

    # --- trim / hover solution --------------------------------------------------------------
    if concept == "stock":
        c = to_norm(base)
        u_trim = rep.step(c[:3], c[3:])[:n]
        st = rep.status()
        trim_exact = bool(st.thrust_setpoint_achieved and st.torque_setpoint_achieved)
        hover_method = f"px4_allocator_CA_METHOD_{ca_method}"
    else:
        u_trim, trim_exact = _hover_bounded_lsq(b, base)
        hover_method = "bounded_min_norm_least_squares"
    achieved_trim = b @ u_trim
    thrust_n = bank.thrust(u_trim)
    power_trim = bank.total_power(u_trim)
    hover = {
        "method": hover_method,
        "u": u_trim,
        "exact": trim_exact,
        "thrust_N": thrust_n,
        "total_thrust_N": float(thrust_n.sum()),
        "vertical_thrust_N": float(-achieved_trim[5]),
        "weight_N": weight_n,
        "residual": achieved_trim - base,
        "power_W": power_trim,
        "power_per_N_W": power_trim / weight_n if weight_n > _EPS else None,
        "headroom": float(1.0 - u_trim.max()),
        "badge": BADGE_STOCK if concept == "stock" else BADGE_FA,
    }
    if not trim_exact:
        notes.append("trim setpoint not exactly achievable at this collective (clipped)")

    # --- authority LPs ----------------------------------------------------------------------
    pinv = np.linalg.pinv(b)
    authority: dict[str, Any] = {}
    auth_ref = np.zeros(6)  # max(|plus|, |minus|) per axis, 0 when unattainable
    auth_plus = np.zeros(6)
    for k in axes:
        entry: dict[str, Any] = {
            "unit": AXIS_UNITS[k],
            "badge": BADGE_STOCK if k in STOCK_AXES else BADGE_FA,
        }
        for sign, tag in ((1, "plus"), (-1, "minus")):
            delta, u_k = _axis_authority(b, base, k, sign)
            attainable = delta is not None and delta > 1e-6
            entry[tag] = float(delta) if delta is not None else None
            entry[f"{tag}_attainable"] = attainable
            if attainable and u_k is not None:
                p_k = bank.total_power(u_k)
                entry[f"{tag}_power_W"] = p_k
                entry[f"{tag}_per_W"] = float(delta) / p_k if p_k > _EPS else None
                entry[f"{tag}_u"] = u_k
            else:
                entry[f"{tag}_power_W"] = None
                entry[f"{tag}_per_W"] = None
                entry[f"{tag}_u"] = None
        plus = entry["plus"] or 0.0
        minus = entry["minus"] or 0.0
        auth_ref[k] = max(plus, minus)
        auth_plus[k] = plus
        # PX4 min-norm cost: largest command change per physical unit (pinv column), and the
        # command needed for the "0.5 N m" reference of PLAN.md M4 on torque axes.
        col = pinv[:, k]
        entry["minnorm_cmd_per_unit"] = float(np.abs(col).max())
        entry["minnorm_fan_of_max"] = int(np.abs(col).argmax())
        if k in TORQUE_AXES:
            entry["minnorm_cmd_per_0p5Nm"] = 0.5 * float(np.abs(col).max())
        authority[AXIS_NAMES[k]] = entry
        if not (entry["plus_attainable"] or entry["minus_attainable"]):
            notes.append(f"{AXIS_NAMES[k]} is unattainable at this collective (LP infeasible)")

    # --- marginal power (finite difference along the min-norm direction) --------------------
    marginal: dict[str, Any] = {}
    for k in axes:
        d = pinv[:, k]
        step = 0.01 * auth_ref[k] if auth_ref[k] > _EPS else 1e-3
        p_plus = bank.total_power(np.clip(u_trim + d * step, 0.0, 1.0))
        p_minus = bank.total_power(np.clip(u_trim - d * step, 0.0, 1.0))
        dp = (p_plus - p_minus) / (2.0 * step)
        marginal[AXIS_NAMES[k]] = {
            "W_per_unit": float(dp),
            "unit": AXIS_UNITS[k],
            "W_per_px4_norm_unit": float(dp / scale[k]) if scale[k] > _EPS else None,
            "step": float(step),
        }

    # --- coupling through the PX4 allocator -------------------------------------------------
    m = len(axes)
    leak_phys = np.zeros((m, 6))
    leak_frac = np.full((m, m), np.nan)
    for i, k in enumerate(axes):
        cmd = np.zeros(6)
        cmd[k] = COUPLING_FRACTION * auth_plus[k]
        c = to_norm(base + cmd)
        u_c = rep.step(c[:3], c[3:])[:n]
        leak_phys[i] = b @ u_c - base
        for j, kk in enumerate(axes):
            if kk == k:
                leak_frac[i, j] = leak_phys[i, kk] / cmd[k] if cmd[k] > _EPS else np.nan
            elif auth_ref[kk] > _EPS:
                leak_frac[i, j] = leak_phys[i, kk] / auth_ref[kk]
    off = leak_frac[~np.eye(m, dtype=bool)]
    off = off[np.isfinite(off)]
    coupling = {
        "axes": [AXIS_NAMES[k] for k in axes],
        "commanded_fraction": COUPLING_FRACTION,
        "commanded": [COUPLING_FRACTION * auth_plus[k] for k in axes],
        "leakage_fraction": leak_frac,
        "leakage_physical": leak_phys,
        "column_axes": list(AXIS_NAMES),
        "column_units": list(AXIS_UNITS),
        "max_offaxis_fraction": float(np.abs(off).max()) if off.size else 0.0,
        "allocator": f"CA_METHOD_{ca_method}",
        "badge": BADGE_STOCK if concept == "stock" else BADGE_FA,
    }

    # --- conditioning -----------------------------------------------------------------------
    bc = b[list(axes)]
    row_max = np.abs(bc).max(axis=1)
    bn = np.where(
        row_max[:, None] > _EPS, bc / np.where(row_max > _EPS, row_max, 1.0)[:, None], 0.0
    )
    sv = np.linalg.svd(bn, compute_uv=False)
    rank = int(np.linalg.matrix_rank(bc))
    conditioning = {
        "axes": [AXIS_NAMES[k] for k in axes],
        "singular_values": sv,
        "rank": rank,
        "full_rank": rank == m,
        "null_space_dim": n - rank,
        "condition_number": float(sv[0] / sv[-1]) if sv[-1] > _EPS else None,
        "px4_scale": scale,
    }

    # --- composite score --------------------------------------------------------------------
    torque_auth = np.array([auth_ref[k] for k in TORQUE_AXES])
    fa_ref = np.array([auth_ref[k] for k in axes])
    normalised = {
        "hover_headroom": float(np.clip(hover["headroom"], 0.0, 1.0)),
        "power_margin": float(np.clip(1.0 - power_trim / bank.max_power(), 0.0, 1.0))
        if bank.max_power() > _EPS
        else 0.0,
        "authority_balance": float(torque_auth.min() / torque_auth.max())
        if torque_auth.max() > _EPS
        else 0.0,
        "decoupling": float(np.clip(1.0 - coupling["max_offaxis_fraction"], 0.0, 1.0)),
        "conditioning": float(sv[-1] / sv[0]) if sv[0] > _EPS else 0.0,
    }
    total_w = sum(w.values())
    score = sum(w[k] * normalised[k] for k in w) / total_w if total_w > _EPS else 0.0
    if fa_ref.min() <= _EPS:
        notes.append("at least one controlled axis has no authority; score is not comparable")

    badges = {"hover": hover["badge"], "coupling": coupling["badge"]}
    badges.update({f"authority.{AXIS_NAMES[k]}": authority[AXIS_NAMES[k]]["badge"] for k in axes})
    badges.update(
        {f"marginal_power.{AXIS_NAMES[k]}": authority[AXIS_NAMES[k]]["badge"] for k in axes}
    )

    estimated_sources = []
    if scenario.mass.estimated:
        estimated_sources.append("mass")
    estimated_sources += [f"fan_curve:{r}" for r, c in bank.curves.items() if c.estimated]

    result = {
        "scenario_name": scenario.meta.name,
        "concept": concept,
        "controlled_axes": [AXIS_NAMES[k] for k in axes],
        "collective": coll,
        "collective_hover": hover_collective,
        "Fz_max_N": fz_max,
        "Fz_target_N": fz_target,
        "px4_thrust_sp_z": float(to_norm(base)[5]),
        "effectiveness": {
            "rows": list(AXIS_NAMES),
            "units": list(AXIS_UNITS),
            "B": b,
            "ct_N": bank.ct,
        },
        "hover": hover,
        "authority": authority,
        "marginal_power": marginal,
        "coupling": coupling,
        "conditioning": conditioning,
        "score": {"value": float(score), "weights": w, "normalised": normalised},
        "badges": badges,
        "estimated": bool(estimated_sources),
        "estimated_sources": estimated_sources,
        "notes": notes,
        "compute_ms": (time.perf_counter() - t_start) * 1e3,
    }
    return _py(result)


def metrics_table(metrics: dict[str, Any]) -> str:
    """Compact fixed-width text table of the headline numbers of one compute_metrics result."""
    h = metrics["hover"]
    header = ("axis", "+auth", "-auth", "unit", "cmd/unit", "dP/du W", "badge")
    lines = [
        f"{metrics['scenario_name']} [{metrics['concept']}] collective {metrics['collective']:.3f}"
        f" (hover {metrics['collective_hover']:.3f}), estimated={metrics['estimated']}",
        f"  hover: headroom {h['headroom']:.3f}, power {h['power_W']:.0f} W,"
        f" max u {max(h['u']):.3f}, exact={h['exact']}",
        f"  {header[0]:<6}{header[1]:>10}{header[2]:>10}{header[3]:>6}{header[4]:>10}"
        f"{header[5]:>10}{header[6]:>34}",
    ]
    for name, a in metrics["authority"].items():
        plus = a["plus"] if a["plus"] is not None else float("nan")
        minus = a["minus"] if a["minus"] is not None else float("nan")
        dp = metrics["marginal_power"][name]["W_per_unit"]
        cost = a["minnorm_cmd_per_unit"]
        lines.append(
            f"  {name:<6}{plus:>10.3f}{minus:>10.3f}{a['unit']:>6}{cost:>10.3f}"
            f"{dp:>10.1f}{a['badge']:>34}"
        )
    c = metrics["conditioning"]
    sv = ", ".join(f"{s:.3f}" for s in c["singular_values"])
    lines.append(
        f"  conditioning: rank {c['rank']}, null dim {c['null_space_dim']}, sv [{sv}],"
        f" max off-axis coupling {metrics['coupling']['max_offaxis_fraction']:.3f},"
        f" score {metrics['score']['value']:.3f}, {metrics['compute_ms']:.1f} ms"
    )
    return "\n".join(lines)
