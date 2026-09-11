"""Rotor geometry and the PX4 control effectiveness matrix.

Port of ActuatorEffectivenessRotors::computeEffectivenessMatrix from the pinned tree
third_party/PX4-Autopilot (v1.17.0, d6f12ad1c4). File tag AER below means
src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp.

Frames and units: positions are metres in the FRD body frame relative to the CG (the same
convention as CA_ROTORn_PX/PY/PZ), axes are FRD vectors along the thrust force, CT is the
thrust of one rotor at u = 1 (N), KM is dimensionless (torque = KM * thrust). Matrix rows are
[roll, pitch, yaw, thrust_x, thrust_y, thrust_z] (ActuatorEffectiveness.hpp:75-85), one
column per actuator. All arithmetic is done in float32 as in PX4; results are returned as
float64 arrays holding exactly the float32 values.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from tiltlab.scenario import Scenario

NUM_ACTUATORS = 16  # ActuatorEffectiveness.hpp:89 (EffectivenessMatrix is 6 x 16)
NUM_AXES = 6
NUM_ROTORS_MAX = 12  # ActuatorEffectivenessRotors.hpp:63
FLT_EPSILON = float(np.finfo(np.float32).eps)

# module.yaml defaults: PX/PY/PZ 0.0 (138,149,160), AX 0.0 (172), AY 0.0 (183), AZ -1.0 (194),
# CT 6.5 (209), KM 0.05 (225), CA_ROTOR_COUNT 0 (127).
DEFAULT_AXIS = (0.0, 0.0, -1.0)
DEFAULT_CT = 6.5
DEFAULT_KM = 0.05


@dataclass(frozen=True)
class Rotor:
    """One rotor as PX4 sees it: position (m, FRD, relative to CG), axis (FRD, along the
    thrust force, not necessarily normalised), thrust_coef CT (N), moment_ratio KM."""

    position: tuple[float, float, float]
    axis: tuple[float, float, float]
    thrust_coef: float
    moment_ratio: float


@dataclass(frozen=True)
class GeometryFlags:
    """Geometry flags of ActuatorEffectivenessRotors.hpp:73-80; all false for CA_AIRFRAME 0
    (ActuatorEffectivenessMultirotor.cpp:38-56 never calls the enable* setters)."""

    propeller_torque_disabled: bool = False
    yaw_by_differential_thrust_disabled: bool = False
    propeller_torque_disabled_non_upwards: bool = False
    three_dimensional_thrust_disabled: bool = False


MULTIROTOR_FLAGS = GeometryFlags()  # CA_AIRFRAME 0: every flag false


def _cross_f32(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    # Vector3.hpp:49-53: {a1*b2 - a2*b1, -a0*b2 + a2*b0, a0*b1 - a1*b0}
    return np.array(
        [
            a[1] * b[2] - a[2] * b[1],
            -a[0] * b[2] + a[2] * b[0],
            a[0] * b[1] - a[1] * b[0],
        ],
        dtype=np.float32,
    )


def compute_effectiveness_matrix(
    rotors: list[Rotor],
    flags: GeometryFlags = MULTIROTOR_FLAGS,
    actuator_start_index: int = 0,
) -> tuple[np.ndarray, int]:
    """Literal port of ActuatorEffectivenessRotors::computeEffectivenessMatrix (AER:145-225).

    Returns (effectiveness 6 x 16 float64, num_actuators). Rotors beyond NUM_ROTORS_MAX are
    ignored by the caller (updateParams, AER:89); this function takes the list as given.
    """
    effectiveness = np.zeros((NUM_AXES, NUM_ACTUATORS), dtype=np.float32)  # Matrix.hpp:24 zero-init
    num_actuators = 0  # AER:148

    for i, rotor in enumerate(rotors):  # AER:150
        if i + actuator_start_index >= NUM_ACTUATORS:  # AER:152-154
            break

        num_actuators += 1  # AER:156 (incremented before the axis and CT checks)

        axis = np.array(rotor.axis, dtype=np.float32)  # AER:159
        axis_norm = np.float32(
            np.sqrt(np.float32(np.dot(axis, axis)))
        )  # AER:162, Vector.hpp:105-109

        if axis_norm > FLT_EPSILON:  # AER:164
            axis = (axis / axis_norm).astype(np.float32)  # AER:165
        else:
            continue  # AER:168-169: bad axis definition, ignore this rotor

        position = np.array(rotor.position, dtype=np.float32)  # AER:173
        ct = np.float32(rotor.thrust_coef)  # AER:176
        km = np.float32(rotor.moment_ratio)  # AER:177

        if flags.propeller_torque_disabled:  # AER:179-181
            km = np.float32(0.0)

        if flags.propeller_torque_disabled_non_upwards:  # AER:183-189
            upwards = abs(axis[0]) < 0.1 and abs(axis[1]) < 0.1 and axis[2] < -0.5
            if not upwards:
                km = np.float32(0.0)

        if abs(ct) < FLT_EPSILON:  # AER:191-193
            continue

        thrust = (ct * axis).astype(np.float32)  # AER:196: thrust = ct * axis
        # AER:199: moment = ct * position.cross(axis) - ct * km * axis
        moment = (ct * _cross_f32(position, axis) - (ct * km) * axis).astype(np.float32)

        col = i + actuator_start_index
        for j in range(3):  # AER:202-205
            effectiveness[j, col] = moment[j]
            effectiveness[j + 3, col] = thrust[j]

        if flags.yaw_by_differential_thrust_disabled:  # AER:207-210
            effectiveness[2, col] = 0.0

        if flags.three_dimensional_thrust_disabled:  # AER:212-221
            effectiveness[3, col] = 0.0
            effectiveness[4, col] = 0.0
            effectiveness[5, col] = -ct

    return effectiveness.astype(np.float64), num_actuators


def effectiveness_matrix(
    positions: np.ndarray,
    axes: np.ndarray,
    ct: np.ndarray | float,
    km: np.ndarray | float,
    flags: GeometryFlags = MULTIROTOR_FLAGS,
) -> np.ndarray:
    """6 x N effectiveness matrix built exactly as PX4 does (see compute_effectiveness_matrix).

    positions: (N, 3) m, FRD, relative to CG. axes: (N, 3) FRD thrust directions (normalised
    inside, AER:162-169). ct: N thrust coefficients (N per unit command) or a scalar. km:
    N moment ratios (dimensionless) or a scalar. N must be <= 16.
    """
    pos = np.asarray(positions, dtype=float).reshape(-1, 3)
    ax = np.asarray(axes, dtype=float).reshape(-1, 3)
    n = pos.shape[0]
    if ax.shape[0] != n:
        raise ValueError("positions and axes must have the same length")
    if n > NUM_ACTUATORS:
        raise ValueError(f"at most {NUM_ACTUATORS} rotors are supported")
    ct_arr = np.broadcast_to(np.asarray(ct, dtype=float), (n,))
    km_arr = np.broadcast_to(np.asarray(km, dtype=float), (n,))
    rotors = [
        Rotor(tuple(pos[i]), tuple(ax[i]), float(ct_arr[i]), float(km_arr[i])) for i in range(n)
    ]
    full, _ = compute_effectiveness_matrix(rotors, flags)
    return full[:, :n]


def rotors_from_px4_params(params: dict[str, float | int]) -> list[Rotor]:
    """Rotor list from a PX4 parameter dict (CA_ROTOR_COUNT, CA_ROTORn_*), as
    ActuatorEffectivenessRotors::updateParams (AER:85-127) with AxisConfiguration::Configurable.
    Missing parameters take the module.yaml defaults."""
    count = int(params.get("CA_ROTOR_COUNT", 0))
    num_rotors = min(NUM_ROTORS_MAX, count)  # AER:89
    rotors: list[Rotor] = []
    for i in range(num_rotors):  # AER:91-126
        p = f"CA_ROTOR{i}_"
        rotors.append(
            Rotor(
                position=(
                    float(params.get(p + "PX", 0.0)),
                    float(params.get(p + "PY", 0.0)),
                    float(params.get(p + "PZ", 0.0)),
                ),
                axis=(
                    float(params.get(p + "AX", DEFAULT_AXIS[0])),
                    float(params.get(p + "AY", DEFAULT_AXIS[1])),
                    float(params.get(p + "AZ", DEFAULT_AXIS[2])),
                ),
                thrust_coef=float(params.get(p + "CT", DEFAULT_CT)),
                moment_ratio=float(params.get(p + "KM", DEFAULT_KM)),
            )
        )
    return rotors


def rotors_from_scenario(scenario: Scenario) -> list[Rotor]:
    """Rotor list from a Scenario, foil-aware: position = effective force point (foil pressure
    point, else fan position) minus mass.cg_frd_m (m, FRD), axis = force direction (motor axis
    turned by the foil deflection when the fan blows into a foil), CT = motor CT (fan curve at
    cmd 1.0 or the CA_ROTORn_CT override) times the foil turning efficiency, KM from the
    reaction-torque toggle. Order is fan id 0..9."""
    rotors: list[Rotor] = []
    for fan in scenario.fans_sorted():
        pos = scenario.hover_pos(fan)  # hover-frame geometry: what PX4's allocator sees
        rotors.append(
            Rotor(
                position=(float(pos[0]), float(pos[1]), float(pos[2])),
                axis=tuple(float(v) for v in scenario.hover_axis(fan)),
                thrust_coef=scenario.fan_ct_effective(fan),
                moment_ratio=scenario.fan_km(fan),
            )
        )
    return rotors
