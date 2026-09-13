import type { Coanda, Fan, Foil, Vec3 } from "./types";

const DEG = Math.PI / 180;

/**
 * Thrust axis in FRD body frame (unit vector) from tilt and azimuth in degrees.
 * PLAN.md section 3: a = (sin t cos p, sin t sin p, -cos t); tilt 0 points up
 * (body -Z), azimuth 0 tilts forward, azimuth 90 tilts right. Mirrors
 * backend/tiltlab/scenario.py tilt_azimuth_to_axis.
 */
export function tiltAzimuthToAxis(tiltDeg: number, azimuthDeg: number): Vec3 {
  const t = tiltDeg * DEG;
  const p = azimuthDeg * DEG;
  return [Math.sin(t) * Math.cos(p), Math.sin(t) * Math.sin(p), -Math.cos(t)];
}

/**
 * Thrust direction after a foil turns the jet down by deflectionDeg about the body lateral axis.
 * Mirrors backend/tiltlab/scenario.py deflect_axis: motor axis (1, 0, 0) gives (cos d, 0, -sin d),
 * so 0 = pure forward thrust, 90 = pure lift, 180 = pure reverse thrust.
 */
export function deflectAxis(motorAxis: Vec3, deflectionDeg: number): Vec3 {
  const d = deflectionDeg * DEG;
  const [x, y, z] = motorAxis;
  return [x * Math.cos(d) + z * Math.sin(d), y, -x * Math.sin(d) + z * Math.cos(d)];
}

export interface EffectiveFan {
  /** Where the force acts (FRD m): the foil pressure point when the fan blows into a foil. */
  pos: Vec3;
  /** Direction of the force on the airframe (FRD unit vector), after the effective turning. */
  axis: Vec3;
  /** Motor axis (FRD unit vector), differs from axis only for foil fans. */
  motorAxis: Vec3;
  /** Foil wrap angle asked for in degrees, or null for a plain fan. */
  deflectionDeg: number | null;
  /** Turning the jet actually gets (Coanda attachment limit applied), or null. */
  effectiveDeg: number | null;
  /** False when the jet separates from the Coanda surface before the requested wrap. */
  attached: boolean;
  /** Coanda separation angle in degrees when the model is on, else null. */
  coandaLimitDeg: number | null;
  /** Fraction of the motor thrust left after the turn. */
  ctScale: number;
}

/** Coanda separation angle, degrees; mirrors backend Coanda.separation_deg. */
export function coandaSeparationDeg(c: Coanda): number {
  return c.theta0_deg * Math.exp((-c.k * c.jet_thickness_m) / c.radius_m);
}

/** Foil-aware geometry of one fan, matching the backend's effective_pos / effective_axis / ct scale. */
export function effectiveFan(fan: Fan, foils: Foil[] | undefined): EffectiveFan {
  const motorAxis = tiltAzimuthToAxis(fan.tilt_deg, fan.azimuth_deg);
  const foil = foils?.find((f) => f.fan_ids.includes(fan.id));
  if (!foil) {
    return { pos: fan.pos_frd_m, axis: motorAxis, motorAxis, deflectionDeg: null, effectiveDeg: null, attached: true, coandaLimitDeg: null, ctScale: 1 };
  }
  const d = foil.per_fan_deflection_deg?.[String(fan.id)] ?? foil.deflection_deg;
  const pos = foil.pressure_points_frd_m?.[String(fan.id)] ?? fan.pos_frd_m;
  const c = foil.coanda;
  if (c && c.enabled) {
    const sep = coandaSeparationDeg(c);
    const attached = d <= sep;
    const eff = attached ? d : sep;
    const scale = Math.max(0, 1 - c.loss_per_90deg * (eff / 90)) * (attached ? 1 : 1 - c.separated_loss);
    return { pos, axis: deflectAxis(motorAxis, eff), motorAxis, deflectionDeg: d, effectiveDeg: eff, attached, coandaLimitDeg: sep, ctScale: scale };
  }
  const s = Math.sin(d * DEG);
  return { pos, axis: deflectAxis(motorAxis, d), motorAxis, deflectionDeg: d, effectiveDeg: d, attached: true, coandaLimitDeg: null, ctScale: 1 - (foil.loss_at_90deg ?? 0) * s * s };
}

/** Plain-words description of a thrust direction, e.g. "up and forward (45 deg from vertical)". */
export function describeAxis(axis: Vec3): string {
  const up = -axis[2];
  const fwd = axis[0];
  const right = axis[1];
  const tilt = (Math.acos(Math.max(-1, Math.min(1, up))) * 180) / Math.PI;
  if (tilt < 0.5) return "straight up";
  const parts: string[] = [];
  if (up > 0.02) parts.push("up");
  if (up < -0.02) parts.push("down");
  if (fwd > 0.02) parts.push("forward");
  if (fwd < -0.02) parts.push("aft");
  if (right > 0.02) parts.push("right");
  if (right < -0.02) parts.push("left");
  return `${parts.join(" and ")} (${tilt.toFixed(0)} deg from vertical)`;
}

/** Mirrored azimuth for a left/right partner fan, degrees in [0, 360). */
export function mirrorAzimuth(azimuthDeg: number): number {
  return ((360 - azimuthDeg) % 360 + 360) % 360;
}

/** Clamp tilt to the valid [0, 90] degree range. */
export function clampTilt(tiltDeg: number): number {
  return Math.min(90, Math.max(0, tiltDeg));
}

/** Wrap azimuth into [0, 360). */
export function wrapAzimuth(azimuthDeg: number): number {
  return ((azimuthDeg % 360) + 360) % 360;
}

/**
 * Map an FRD vector (metres) to three.js scene coordinates (Y up):
 * scene.x = FRD x (forward), scene.y = -FRD z (up), scene.z = FRD y (right).
 * The mapping is a proper rotation so handedness is preserved.
 */
export function frdToScene(v: Vec3): Vec3 {
  return [v[0], -v[2], v[1]];
}

/**
 * Forward and side tilt of a thrust axis, degrees: the lean seen from the side (positive forward,
 * negative aft) and the lean seen from the front (positive right, negative left). Each is the
 * angle from vertical of the axis projected onto its own plane, so the two are independent and
 * match the two angles a mount is drawn and printed with. axis ~ (tan fwd, tan side, -1).
 */
export function tiltAzimuthToFwdSide(tiltDeg: number, azimuthDeg: number): { fwd: number; side: number } {
  const t = Math.min(tiltDeg, 89.9) * DEG;
  const p = azimuthDeg * DEG;
  return {
    fwd: Math.atan(Math.tan(t) * Math.cos(p)) / DEG,
    side: Math.atan(Math.tan(t) * Math.sin(p)) / DEG,
  };
}

/** Inverse of tiltAzimuthToFwdSide; tilt in [0, 90), azimuth in [0, 360). */
export function fwdSideToTiltAzimuth(fwdDeg: number, sideDeg: number): { tilt: number; azimuth: number } {
  const tf = Math.tan(clampSideTilt(fwdDeg) * DEG);
  const ts = Math.tan(clampSideTilt(sideDeg) * DEG);
  const tilt = Math.atan(Math.hypot(tf, ts)) / DEG;
  const azimuth = tilt < 1e-9 ? 0 : wrapAzimuth(Math.atan2(ts, tf) / DEG);
  return { tilt: Math.round(tilt * 1e6) / 1e6, azimuth: Math.round(azimuth * 1e6) / 1e6 };
}

/** Forward and side tilt stay short of 90 so the tangent stays finite. */
export function clampSideTilt(deg: number): number {
  return Math.min(89, Math.max(-89, Number.isFinite(deg) ? deg : 0));
}
