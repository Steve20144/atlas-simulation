"""Fan (EDF) model: thrust and power curves, inverse, first-order lag, battery sag.

Everything here is scalar physics in SI units. Commands (cmd) are the dimensionless PX4
actuator outputs in [0, 1] (actuator_motors.control). Thrust is in newtons along the fan
axis (the frame of the axis is the caller's business, see geometry.py). Power is electrical
input power in watts, voltage in volts, current in amperes, charge in ampere hours where the
name says so and in coulombs otherwise, time in seconds.

This module deliberately does not import scenario.py or the other core modules; it accepts
the plain ``fan_curves`` entry dict from PLAN.md section 5 and returns the same shape.
Every curve carries ``estimated`` so the UI can flag curves that were not measured.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

G0 = 9.80665  # m/s^2, standard gravity (PLAN.md section 5, environment.gravity default)

# Published XFly 80 mm 12-blade points (PLAN.md section 1, manufacturer listings). Only the
# maximum static thrust and the current at that point are known.
XFLY80_NOMINAL_PACK_V = 22.2  # V, 6S at 3.7 V per cell
XFLY80_VARIANTS: dict[str, dict[str, Any]] = {
    "3280-KV2200": {
        "ref": "xfly80_3280",
        "max_thrust_kg": 3.400,
        "max_current_a": 100.0,
        "cells": 6,
    },
    "3665-KV2300 PRO": {
        "ref": "xfly80_3665pro",
        "max_thrust_kg": 3.650,
        "max_current_a": 120.0,
        "cells": 6,
    },
}
_XFLY80_ALIASES = {
    "3280": "3280-KV2200",
    "3280-kv2200": "3280-KV2200",
    "xfly80_3280": "3280-KV2200",
    "3665": "3665-KV2300 PRO",
    "3665-kv2300": "3665-KV2300 PRO",
    "3665-kv2300 pro": "3665-KV2300 PRO",
    "3665-kv2300-pro": "3665-KV2300 PRO",
    "xfly80_3665pro": "3665-KV2300 PRO",
    "xfly80_3665": "3665-KV2300 PRO",
}
XFLY80_DEFAULT_LAG_S = 0.15  # s, PLAN.md section 5 example; not measured
XFLY80_ESTIMATED_INTERIOR_POINTS = 6


def _as_array(x: ArrayLike) -> NDArray[np.float64]:
    return np.asarray(x, dtype=np.float64)


def _opt_float(x: float | None) -> float | None:
    return None if x is None else float(x)


@dataclass(frozen=True)
class FanCurve:
    """Static thrust (N) and electrical power (W) of one fan versus command in [0, 1].

    Points are piecewise-linear knots; evaluation clamps the command to [cmd[0], cmd[-1]].
    ``cells`` is the pack cell count the curve was measured (or estimated) at.
    ``rotor_inertia_kgm2`` (kg m^2, about the fan axis) is the hook for the gyroscopic
    term in the rigid body model; None means "not modelled".
    """

    cmd: NDArray[np.float64]
    thrust_n: NDArray[np.float64]
    power_w: NDArray[np.float64]
    cells: int = 6
    lag_s: float = XFLY80_DEFAULT_LAG_S
    max_continuous_a: float | None = None
    notes: str = ""
    estimated: bool = False
    rotor_inertia_kgm2: float | None = None

    def __post_init__(self) -> None:
        cmd = _as_array(self.cmd).ravel()
        thrust = _as_array(self.thrust_n).ravel()
        power = _as_array(self.power_w).ravel()
        if not (cmd.shape == thrust.shape == power.shape):
            raise ValueError("cmd, thrust_N and power_W must have the same length")
        if cmd.size < 2:
            raise ValueError("a fan curve needs at least two points")
        if not all(np.all(np.isfinite(a)) for a in (cmd, thrust, power)):
            raise ValueError("fan curve points must be finite")
        if np.any(cmd < 0.0) or np.any(cmd > 1.0):
            raise ValueError("cmd must lie in [0, 1]")
        if np.any(np.diff(cmd) <= 0.0):
            raise ValueError("cmd must be strictly increasing (sorted, no duplicates)")
        if np.any(np.diff(thrust) < 0.0):
            raise ValueError("thrust_N must be non-decreasing with cmd")
        if np.any(np.diff(power) < 0.0):
            raise ValueError("power_W must be non-decreasing with cmd")
        if np.any(thrust < 0.0) or np.any(power < 0.0):
            raise ValueError("thrust_N and power_W must be non-negative")
        if self.lag_s < 0.0:
            raise ValueError("lag_s must be >= 0")
        if self.cells < 1:
            raise ValueError("cells must be >= 1")
        # Frozen dataclass: store the validated arrays via object.__setattr__.
        object.__setattr__(self, "cmd", cmd)
        object.__setattr__(self, "thrust_n", thrust)
        object.__setattr__(self, "power_w", power)

    # -- construction -----------------------------------------------------------------

    @classmethod
    def from_points(
        cls,
        points: Iterable[Mapping[str, float]],
        *,
        cells: int = 6,
        lag_s: float = XFLY80_DEFAULT_LAG_S,
        max_continuous_a: float | None = None,
        notes: str = "",
        estimated: bool = False,
        rotor_inertia_kgm2: float | None = None,
    ) -> FanCurve:
        """Build from PLAN.md section 5 points: [{"cmd", "thrust_N", "power_W"}, ...]."""
        pts = list(points)
        if not pts:
            raise ValueError("points is empty")
        cmd = np.array([float(p["cmd"]) for p in pts])
        thrust = np.array([float(p["thrust_N"]) for p in pts])
        power = np.array([float(p["power_W"]) for p in pts])
        return cls(
            cmd=cmd,
            thrust_n=thrust,
            power_w=power,
            cells=int(cells),
            lag_s=float(lag_s),
            max_continuous_a=None if max_continuous_a is None else float(max_continuous_a),
            notes=str(notes),
            estimated=bool(estimated),
            rotor_inertia_kgm2=None if rotor_inertia_kgm2 is None else float(rotor_inertia_kgm2),
        )

    @classmethod
    def from_scenario_curve(cls, curve: Mapping[str, Any]) -> FanCurve:
        """Build from one ``fan_curves`` entry of the scenario JSON (PLAN.md section 5).

        Keys: cells, points, lag_s, max_continuous_A, notes, estimated, rotor_inertia_kgm2.
        Missing optional keys take the defaults; a missing ``estimated`` is False.
        """
        return cls.from_points(
            curve["points"],
            cells=curve.get("cells", 6),
            lag_s=curve.get("lag_s", XFLY80_DEFAULT_LAG_S),
            max_continuous_a=curve.get("max_continuous_A"),
            notes=curve.get("notes", ""),
            estimated=curve.get("estimated", False),
            rotor_inertia_kgm2=curve.get("rotor_inertia_kgm2"),
        )

    def to_scenario_curve(self) -> dict[str, Any]:
        """Return the PLAN.md section 5 ``fan_curves`` entry dict (JSON-serialisable)."""
        return {
            "cells": int(self.cells),
            "points": [
                {"cmd": float(c), "thrust_N": float(t), "power_W": float(p)}
                for c, t, p in zip(self.cmd, self.thrust_n, self.power_w, strict=True)
            ],
            "lag_s": float(self.lag_s),
            "max_continuous_A": _opt_float(self.max_continuous_a),
            "notes": self.notes,
            "estimated": bool(self.estimated),
            "rotor_inertia_kgm2": _opt_float(self.rotor_inertia_kgm2),
        }

    # -- evaluation -------------------------------------------------------------------

    def thrust(self, cmd: ArrayLike) -> NDArray[np.float64]:
        """Static thrust (N) at command cmd (dimensionless, clamped to the knot range)."""
        return np.interp(_as_array(cmd), self.cmd, self.thrust_n)

    def power(self, cmd: ArrayLike) -> NDArray[np.float64]:
        """Electrical input power (W) at command cmd (dimensionless, clamped)."""
        return np.interp(_as_array(cmd), self.cmd, self.power_w)

    def cmd_from_thrust(self, thrust_n: ArrayLike) -> NDArray[np.float64]:
        """Inverse: the command (dimensionless) producing thrust_n (N), piecewise linear.

        Thrust outside [thrust(cmd[0]), thrust(cmd[-1])] clamps to the end commands. Where
        the curve is flat (a deadband) the highest command of the flat segment (the start of
        the rising part) is returned, so thrust(cmd_from_thrust(t)) == t for every reachable t.
        """
        keep = np.concatenate((np.diff(self.thrust_n) > 0.0, [True]))
        return np.interp(_as_array(thrust_n), self.thrust_n[keep], self.cmd[keep])

    def ct_for_px4(self) -> float:
        """CA_ROTORn_CT (N): thrust at cmd = 1.0 (PLAN.md section 5, "derived")."""
        return float(self.thrust(1.0))

    @property
    def max_thrust_n(self) -> float:
        """Largest thrust on the curve (N)."""
        return float(self.thrust_n[-1])

    @property
    def max_power_w(self) -> float:
        """Largest electrical power on the curve (W)."""
        return float(self.power_w[-1])


def estimated_default_xfly80(variant: str = "3280-KV2200") -> FanCurve:
    """Default XFly 80 mm curve from the published maximum point, flagged estimated.

    Variants (PLAN.md section 1): "3280-KV2200" (3400 g at about 100 A on 6S) and
    "3665-KV2300 PRO" (3650 g at about 120 A on 6S). Short aliases ("3280", "3665",
    "xfly80_3280", ...) are accepted.

    Assumptions (replace with thrust-stand data as soon as it exists):
    - Maximum power P_max = I_max * 22.2 V (6S nominal pack voltage), W.
    - Maximum thrust T_max = m_max * g0, N.
    - Momentum theory: thrust proportional to power^(2/3). The command is taken as
      proportional to fan speed, so power = P_max * cmd^3 and thrust = T_max * cmd^2,
      which satisfies thrust proportional to power^(2/3) exactly at every knot.
    - Knots at cmd = 0, six interior points equally spaced, and cmd = 1 (8 points).
    """
    key = _XFLY80_ALIASES.get(variant.strip().lower(), variant.strip())
    if key not in XFLY80_VARIANTS:
        raise ValueError(f"unknown XFly 80 variant {variant!r}; known: {sorted(XFLY80_VARIANTS)}")
    spec = XFLY80_VARIANTS[key]
    t_max = spec["max_thrust_kg"] * G0
    p_max = spec["max_current_a"] * XFLY80_NOMINAL_PACK_V
    cmd = np.linspace(0.0, 1.0, XFLY80_ESTIMATED_INTERIOR_POINTS + 2)
    power = p_max * cmd**3
    thrust = t_max * (power / p_max) ** (2.0 / 3.0)
    thrust[-1] = t_max  # remove the last-digit rounding of the fractional power
    return FanCurve(
        cmd=cmd,
        thrust_n=thrust,
        power_w=power,
        cells=int(spec["cells"]),
        lag_s=XFLY80_DEFAULT_LAG_S,
        max_continuous_a=float(spec["max_current_a"]),
        notes=(
            f"XFly 80 mm 12-blade {key}: estimated from the manufacturer max point "
            f"({spec['max_thrust_kg'] * 1000:.0f} g at {spec['max_current_a']:.0f} A on "
            f"{spec['cells']}S, {XFLY80_NOMINAL_PACK_V} V nominal) with thrust proportional to "
            "power^(2/3); replace with thrust stand data"
        ),
        estimated=True,
    )


@dataclass
class FanLag:
    """First-order lag between commanded and effective fan command.

    x_{k+1} = x_k + (u - x_k) * (1 - exp(-dt / tau)), the exact solution of
    tau * dx/dt = u - x for a command u held constant over the step. tau_s = 0 means no lag.
    State is dimensionless (same units as cmd) and may be a scalar or an array (one fan
    per element).
    """

    tau_s: float
    state: NDArray[np.float64] = field(default_factory=lambda: np.zeros(()))

    def __post_init__(self) -> None:
        if self.tau_s < 0.0:
            raise ValueError("tau_s must be >= 0")
        self.state = _as_array(self.state).copy()

    def reset(self, cmd: ArrayLike = 0.0) -> None:
        """Set the internal state to cmd (dimensionless)."""
        self.state = _as_array(cmd).copy()

    def step(self, cmd: ArrayLike, dt_s: float) -> NDArray[np.float64]:
        """Advance by dt_s (s) with commanded cmd held constant; return the new state."""
        if dt_s < 0.0:
            raise ValueError("dt_s must be >= 0")
        u = _as_array(cmd)
        if self.tau_s == 0.0:
            self.state = u.copy()
        else:
            alpha = 1.0 - np.exp(-dt_s / self.tau_s)
            self.state = self.state + (u - self.state) * alpha
        return self.state


@dataclass
class Battery:
    """Simple LiPo pack: linear open-circuit voltage versus state of charge plus an ohmic drop.

    ``cells`` in series; ``cell_full_v`` and ``cell_nominal_v`` in V per cell;
    ``cell_resistance_ohm`` in ohm per cell (pack resistance = cells * cell_resistance_ohm);
    ``capacity_ah`` in A h; ``consumed_ah`` (state) in A h.

    Assumptions:
    - Open-circuit cell voltage falls linearly with depth of discharge from ``cell_full_v``
      at full charge to (2 * nominal - full) at empty, so it equals ``cell_nominal_v`` at
      50 percent state of charge (4.2 V full and 3.7 V nominal give 3.2 V empty).
    - Terminal voltage under load: V = OCV - I * R_pack (no dynamic terms).
    - Optional thrust derating: fan speed is proportional to voltage at fixed command
      (KV rating) and thrust is proportional to speed squared, so thrust scales with
      (V / V_nominal_pack)^2. The factor is not clipped at 1, so a full pack gives more
      than the curve's nominal thrust; the fan curve is assumed to be at nominal voltage.
    """

    cells: int = 6
    cell_full_v: float = 4.2
    cell_nominal_v: float = 3.7
    cell_resistance_ohm: float = 0.003
    capacity_ah: float = 10.0
    consumed_ah: float = 0.0

    def __post_init__(self) -> None:
        if self.cells < 1:
            raise ValueError("cells must be >= 1")
        if self.cell_full_v <= 0.0 or self.cell_nominal_v <= 0.0:
            raise ValueError("cell voltages must be positive")
        if self.cell_full_v < self.cell_nominal_v:
            raise ValueError("cell_full_v must be >= cell_nominal_v")
        if self.cell_resistance_ohm < 0.0:
            raise ValueError("cell_resistance_ohm must be >= 0")
        if self.capacity_ah <= 0.0:
            raise ValueError("capacity_ah must be positive")

    @property
    def nominal_pack_v(self) -> float:
        """Nominal pack voltage (V) = cells * cell_nominal_v."""
        return self.cells * self.cell_nominal_v

    @property
    def full_pack_v(self) -> float:
        """Full-charge pack voltage (V) = cells * cell_full_v."""
        return self.cells * self.cell_full_v

    @property
    def pack_resistance_ohm(self) -> float:
        """Pack internal resistance (ohm) = cells * cell_resistance_ohm."""
        return self.cells * self.cell_resistance_ohm

    @property
    def state_of_charge(self) -> float:
        """Remaining fraction of capacity (dimensionless, clipped to [0, 1])."""
        return float(np.clip(1.0 - self.consumed_ah / self.capacity_ah, 0.0, 1.0))

    @property
    def consumed_c(self) -> float:
        """Consumed charge (C) = consumed_ah * 3600."""
        return self.consumed_ah * 3600.0

    def open_circuit_voltage(self) -> float:
        """Pack open-circuit voltage (V) at the current state of charge."""
        cell_empty_v = 2.0 * self.cell_nominal_v - self.cell_full_v
        cell_v = cell_empty_v + (self.cell_full_v - cell_empty_v) * self.state_of_charge
        return self.cells * cell_v

    def voltage_under_load(self, current_a: ArrayLike) -> NDArray[np.float64]:
        """Terminal pack voltage (V) when drawing current_a (A): OCV - I * R_pack."""
        return self.open_circuit_voltage() - _as_array(current_a) * self.pack_resistance_ohm

    @staticmethod
    def current_from_power(power_w: ArrayLike, voltage_v: ArrayLike) -> NDArray[np.float64]:
        """Current (A) = power_w (W) / voltage_v (V)."""
        return _as_array(power_w) / _as_array(voltage_v)

    def load(self, power_w: ArrayLike) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Self-consistent (voltage V, current A) for a total electrical load power_w (W).

        Solves V = OCV - R * P / V, i.e. V^2 - OCV * V + R * P = 0, taking the upper root.
        If P exceeds the maximum deliverable power OCV^2 / (4 R) the voltage is pinned at
        OCV / 2 and the current at OCV / (2 R) (the pack cannot supply that power).
        """
        p = _as_array(power_w)
        ocv = self.open_circuit_voltage()
        r = self.pack_resistance_ohm
        if r == 0.0:
            v = np.full_like(p, ocv, dtype=np.float64)
            return v, p / v
        disc = np.maximum(ocv * ocv - 4.0 * r * p, 0.0)
        v = 0.5 * (ocv + np.sqrt(disc))
        return v, (ocv - v) / r

    def consume(self, current_a: float, dt_s: float) -> float:
        """Integrate charge: consumed_ah += current_a (A) * dt_s (s) / 3600; returns consumed_ah."""
        if dt_s < 0.0:
            raise ValueError("dt_s must be >= 0")
        self.consumed_ah += float(current_a) * dt_s / 3600.0
        return self.consumed_ah

    def thrust_derating(self, voltage_v: ArrayLike) -> NDArray[np.float64]:
        """Thrust scale factor (dimensionless) = (voltage_v / nominal_pack_v)^2, floored at 0.

        Assumption documented in the class docstring: speed proportional to voltage at fixed
        command, thrust proportional to speed squared.
        """
        v = np.maximum(_as_array(voltage_v), 0.0)
        return (v / self.nominal_pack_v) ** 2

    def derated_thrust(
        self, curve: FanCurve, cmd: ArrayLike, voltage_v: ArrayLike
    ) -> NDArray[np.float64]:
        """Thrust (N) from curve at cmd scaled by thrust_derating(voltage_v (V))."""
        return curve.thrust(cmd) * self.thrust_derating(voltage_v)


__all__ = [
    "G0",
    "XFLY80_NOMINAL_PACK_V",
    "XFLY80_VARIANTS",
    "Battery",
    "FanCurve",
    "FanLag",
    "estimated_default_xfly80",
]
