"""Scenario data model (PLAN.md section 5) and the tilt/azimuth <-> axis helpers.

Frames and units: every position is FRD body frame in metres, every axis is an FRD unit
vector pointing in the direction of the thrust force (a rotor lifting the vehicle has
axis (0, 0, -1)). Angles are stored in degrees because the Scenario is a UI-facing
document; all physics code converts to radians internally.
"""

from __future__ import annotations

import math
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

NUM_FANS = 10

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


def tilt_azimuth_to_axis(tilt_deg: float, azimuth_deg: float) -> np.ndarray:
    """Thrust axis (FRD unit vector, dimensionless) from tilt and azimuth in degrees.

    tilt is the angle between the thrust axis and straight up (-Z FRD); azimuth is the
    heading of the tilted component measured from +X (forward) towards +Y (right).
    axis = (sin t cos p, sin t sin p, -cos t).
    """
    t = math.radians(tilt_deg)
    p = math.radians(azimuth_deg)
    axis = np.array([math.sin(t) * math.cos(p), math.sin(t) * math.sin(p), -math.cos(t)])
    axis[np.abs(axis) < 1e-12] = 0.0  # cos(90 deg) etc. are exact zeros, not 6e-17
    return axis


def deflect_axis(motor_axis: np.ndarray | tuple[float, ...], deflection_deg: float) -> np.ndarray:
    """Thrust direction after a foil turns the jet down by deflection_deg (FRD unit vector).

    The foil rotates the jet about the body lateral axis (+Y), so the reaction force rotates the
    same way: a motor whose thrust axis is (1, 0, 0) (blowing aft) gives
    (cos d, 0, -sin d): 0 deg is pure forward thrust, 90 deg is pure lift (0, 0, -1),
    180 deg is pure reverse thrust.
    """
    d = math.radians(deflection_deg)
    x, y, z = (float(v) for v in motor_axis)
    out = np.array([x * math.cos(d) + z * math.sin(d), y, -x * math.sin(d) + z * math.cos(d)])
    out[np.abs(out) < 1e-12] = 0.0
    return out


def axis_to_tilt_azimuth(axis: np.ndarray | list[float] | tuple[float, ...]) -> tuple[float, float]:
    """Inverse of tilt_azimuth_to_axis. axis is any non-zero FRD vector (normalised here).

    Returns (tilt_deg, azimuth_deg) with azimuth in [0, 360) and azimuth 0 when tilt is 0.
    """
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n <= 0.0:
        raise ValueError("axis must be non-zero")
    a = a / n
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -a[2]))))
    if math.hypot(a[0], a[1]) < 1e-9:
        return (0.0 if tilt < 1e-6 else tilt, 0.0)
    az = math.degrees(math.atan2(a[1], a[0])) % 360.0
    return (tilt, az)


class Meta(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str
    created: str
    px4_version: str = "1.17.0"
    notes: str = ""


class Frame(BaseModel):
    model_config = ConfigDict(extra="forbid")
    cad_forward_axis: str = "+X"
    cad_up_axis: str = "+Z"
    cad_units: Literal["mm", "m", "in"] = "mm"


class CadReported(BaseModel):
    model_config = ConfigDict(extra="forbid")
    mass_kg: float = 0.0
    cg_m: Vec3 = (0.0, 0.0, 0.0)


class Body(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = ""
    volume_m3: float = 0.0
    mass_kg: float = 0.0
    source: Literal["user", "density", "cad"] = "user"


class Mass(BaseModel):
    """total_kg in kg; cg_frd_m in metres, FRD body frame; inertia about the CG in kg m^2, FRD."""

    model_config = ConfigDict(extra="forbid")
    total_kg: float = Field(ge=0.0)
    cg_frd_m: Vec3 = (0.0, 0.0, 0.0)
    inertia_frd_kgm2: Mat3 = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    cad_reported: CadReported | None = None
    bodies: list[Body] = Field(default_factory=list)
    estimated: bool = False
    notes: str = ""


class Fan(BaseModel):
    """One electric ducted fan.

    pos_frd_m: fan thrust application point, metres, FRD body frame, relative to the body
    origin (the CA_ROTORn_P* values are pos_frd_m minus mass.cg_frd_m).
    tilt_deg / azimuth_deg: see tilt_azimuth_to_axis. km: magnitude of the PX4 moment
    coefficient (Torque = KM * Thrust, dimensionless); its sign is derived from spin
    (positive for CCW, module.yaml:211-225) when control.reaction_torque is on.
    """

    model_config = ConfigDict(extra="forbid")
    id: int = Field(ge=0, le=NUM_FANS - 1)
    output: str = ""
    pos_frd_m: Vec3
    tilt_deg: float = Field(ge=0.0, le=90.0)
    azimuth_deg: float = Field(ge=0.0, le=360.0)
    spin: Literal["CW", "CCW"] = "CW"
    mirror_of: int | None = None
    curve_ref: str
    km: float = Field(default=0.0, ge=0.0)

    @model_validator(mode="after")
    def _default_output(self) -> Fan:
        if not self.output:
            self.output = f"MAIN{self.id + 1}"
        return self

    def axis(self) -> np.ndarray:
        """FRD unit thrust axis (dimensionless)."""
        return tilt_azimuth_to_axis(self.tilt_deg, self.azimuth_deg)

    def signed_km(self) -> float:
        return self.km if self.spin == "CCW" else -self.km


class Foil(BaseModel):
    """A jet-deflecting foil behind a group of motors.

    The motors in fan_ids blow into the foil (their Fan.tilt_deg/azimuth_deg describe the MOTOR
    thrust axis; 90 / 0 is a horizontal motor blowing aft). The foil turns the jet down by
    deflection_deg about the body lateral axis: 0 leaves the jet straight aft (pure forward
    thrust), 90 sends it straight down (pure lift), 90 to 180 sends it forward (reverse thrust).
    The force on the airframe acts at pressure_points_frd_m (per fan id; FRD metres, same origin as
    Fan.pos_frd_m) along deflect_axis(motor axis, deflection) and its magnitude is the motor thrust
    times 1 - loss_at_90deg * sin^2(deflection). per_fan_deflection_deg overrides deflection_deg for
    a segmented foil.
    """

    model_config = ConfigDict(extra="forbid")
    id: str
    fan_ids: list[int] = Field(min_length=1)
    deflection_deg: float = Field(default=45.0, ge=0.0, le=180.0)
    per_fan_deflection_deg: dict[int, float] = Field(default_factory=dict)
    pressure_points_frd_m: dict[int, Vec3] = Field(default_factory=dict)
    loss_at_90deg: float = Field(default=0.0, ge=0.0, le=1.0)
    estimated: bool = True
    notes: str = ""

    @model_validator(mode="after")
    def _check_ids(self) -> Foil:
        for fid, d in self.per_fan_deflection_deg.items():
            if fid not in self.fan_ids:
                raise ValueError(f"foil {self.id}: per_fan_deflection for fan {fid} not in fan_ids")
            if not 0.0 <= d <= 180.0:
                raise ValueError(f"foil {self.id}: deflection {d} out of [0, 180]")
        for fid in self.pressure_points_frd_m:
            if fid not in self.fan_ids:
                raise ValueError(f"foil {self.id}: pressure point for fan {fid} not in fan_ids")
        return self

    def deflection_for(self, fan_id: int) -> float:
        return float(self.per_fan_deflection_deg.get(fan_id, self.deflection_deg))

    def ct_scale(self, fan_id: int) -> float:
        """Fraction of the motor thrust that survives the turn (dimensionless)."""
        s = math.sin(math.radians(self.deflection_for(fan_id)))
        return 1.0 - self.loss_at_90deg * s * s


class CurvePoint(BaseModel):
    model_config = ConfigDict(extra="forbid")
    cmd: float = Field(ge=0.0, le=1.0)
    thrust_N: float = Field(ge=0.0)
    power_W: float = Field(ge=0.0)


class FanCurve(BaseModel):
    """Static fan curve: thrust (N) and electrical power (W) versus normalised command."""

    model_config = ConfigDict(extra="forbid")
    cells: int = Field(ge=1)
    points: list[CurvePoint] = Field(min_length=2)
    lag_s: float = Field(ge=0.0)
    max_continuous_A: float = Field(ge=0.0)
    notes: str = ""
    estimated: bool = False

    @field_validator("points")
    @classmethod
    def _monotonic_cmd(cls, pts: list[CurvePoint]) -> list[CurvePoint]:
        cmds = [p.cmd for p in pts]
        if cmds != sorted(cmds) or len(set(cmds)) != len(cmds):
            raise ValueError("fan curve points must have strictly increasing cmd")
        return pts

    def thrust_at(self, cmd: float) -> float:
        """Thrust in N at normalised command cmd (linear interpolation, clamped)."""
        c = [p.cmd for p in self.points]
        t = [p.thrust_N for p in self.points]
        return float(np.interp(cmd, c, t))

    def power_at(self, cmd: float) -> float:
        """Electrical power in W at normalised command cmd (linear interpolation, clamped)."""
        c = [p.cmd for p in self.points]
        w = [p.power_W for p in self.points]
        return float(np.interp(cmd, c, w))


class Control(BaseModel):
    model_config = ConfigDict(extra="forbid")
    concept: Literal["stock", "fully_actuated"] = "stock"
    blend: float = Field(default=0.0, ge=0.0, le=1.0)
    ca_method: Literal[0, 1, 2] = 2
    reaction_torque: bool = False
    px4_params_override: dict[str, float | int] = Field(default_factory=dict)


class Rig(BaseModel):
    """attitude_offset_deg is [roll, pitch, yaw] in degrees, body FRD relative to NED level."""

    model_config = ConfigDict(extra="forbid")
    enabled: bool = True
    lock_position: bool = True
    lock_roll: bool = True
    lock_pitch: bool = False
    lock_yaw: bool = False
    attitude_offset_deg: Vec3 = (0.0, 0.0, 0.0)


class Environment(BaseModel):
    """air_density kg/m^3, wind_ned_mps m/s in NED, gravity m/s^2."""

    model_config = ConfigDict(extra="forbid")
    air_density: float = 1.225
    wind_ned_mps: Vec3 = (0.0, 0.0, 0.0)
    gravity: float = 9.80665


class Outputs(BaseModel):
    model_config = ConfigDict(extra="forbid")
    params: bool = True
    csv: bool = True
    report: bool = True
    plots: bool = True
    sdf: bool = False
    angle_sheet: bool = True


class Scenario(BaseModel):
    model_config = ConfigDict(extra="forbid")
    meta: Meta
    frame: Frame = Field(default_factory=Frame)
    mass: Mass
    fans: list[Fan]
    fan_curves: dict[str, FanCurve]
    control: Control = Field(default_factory=Control)
    rig: Rig = Field(default_factory=Rig)
    environment: Environment = Field(default_factory=Environment)
    outputs: Outputs = Field(default_factory=Outputs)
    foils: list[Foil] = Field(default_factory=list)

    @model_validator(mode="after")
    def _check_fans(self) -> Scenario:
        if len(self.fans) != NUM_FANS:
            raise ValueError(f"exactly {NUM_FANS} fans required, got {len(self.fans)}")
        ids = sorted(f.id for f in self.fans)
        if ids != list(range(NUM_FANS)):
            raise ValueError(f"fan ids must be 0..{NUM_FANS - 1} exactly once, got {ids}")
        for f in self.fans:
            if f.curve_ref not in self.fan_curves:
                raise ValueError(f"fan {f.id}: curve_ref '{f.curve_ref}' not in fan_curves")
            if f.mirror_of is not None and f.mirror_of not in ids:
                raise ValueError(f"fan {f.id}: mirror_of {f.mirror_of} is not a fan id")
        seen: set[int] = set()
        for foil in self.foils:
            for fid in foil.fan_ids:
                if fid not in ids:
                    raise ValueError(f"foil {foil.id}: fan {fid} is not a fan id")
                if fid in seen:
                    raise ValueError(f"fan {fid} belongs to more than one foil")
                seen.add(fid)
        return self

    def fans_sorted(self) -> list[Fan]:
        return sorted(self.fans, key=lambda f: f.id)

    # ---- effective geometry (foil-aware): what the airframe feels -------------------------

    def foil_for(self, fan_id: int) -> Foil | None:
        for foil in self.foils:
            if fan_id in foil.fan_ids:
                return foil
        return None

    def effective_axis(self, fan: Fan) -> np.ndarray:
        """FRD unit direction of the force on the airframe: the motor axis, turned by the foil
        deflection when the fan blows into a foil."""
        foil = self.foil_for(fan.id)
        if foil is None:
            return fan.axis()
        return deflect_axis(fan.axis(), foil.deflection_for(fan.id))

    def effective_pos(self, fan: Fan) -> np.ndarray:
        """FRD point (m, body origin) where the force acts: the foil pressure point when the fan
        blows into a foil that has one recorded, else the fan position."""
        foil = self.foil_for(fan.id)
        if foil is not None and fan.id in foil.pressure_points_frd_m:
            return np.asarray(foil.pressure_points_frd_m[fan.id], dtype=float)
        return np.asarray(fan.pos_frd_m, dtype=float)

    def foil_ct_scale(self, fan: Fan) -> float:
        foil = self.foil_for(fan.id)
        return 1.0 if foil is None else foil.ct_scale(fan.id)

    def fan_ct_effective(self, fan: Fan) -> float:
        """CA_ROTORn_CT the allocator must see (N): motor CT times the foil turning efficiency."""
        return self.fan_ct(fan) * self.foil_ct_scale(fan)

    def fan_deflection(self, fan: Fan) -> float | None:
        foil = self.foil_for(fan.id)
        return None if foil is None else foil.deflection_for(fan.id)

    def fan_ct(self, fan: Fan) -> float:
        """PX4 CA_ROTORn_CT for a fan: curve thrust (N) at cmd 1.0, unless overridden
        by control.px4_params_override['CA_ROTOR<n>_CT']."""
        key = f"CA_ROTOR{fan.id}_CT"
        if key in self.control.px4_params_override:
            return float(self.control.px4_params_override[key])
        return self.fan_curves[fan.curve_ref].thrust_at(1.0)

    def fan_km(self, fan: Fan) -> float:
        """PX4 CA_ROTORn_KM (signed, dimensionless): 0 unless control.reaction_torque."""
        return fan.signed_km() if self.control.reaction_torque else 0.0
