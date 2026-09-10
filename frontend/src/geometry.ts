import type { Vec3 } from "./types";

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
