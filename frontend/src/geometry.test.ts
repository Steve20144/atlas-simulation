import { describe, expect, it } from "vitest";
import { deflectAxis, describeAxis, effectiveFan, tiltAzimuthToAxis, fwdSideToTiltAzimuth, tiltAzimuthToFwdSide, signedTilt, clampTilt, pastHorizontal } from "./geometry";
import type { Fan, Foil } from "./types";

const close = (a: number[], b: number[]) => a.every((v, i) => Math.abs(v - b[i]) < 1e-6);

describe("foil geometry", () => {
  const motor: [number, number, number] = tiltAzimuthToAxis(90, 0); // horizontal, blowing aft

  it("deflects the thrust of a horizontal motor about the lateral axis", () => {
    expect(close(motor, [1, 0, 0])).toBe(true);
    expect(close(deflectAxis(motor, 0), [1, 0, 0])).toBe(true);
    expect(close(deflectAxis(motor, 45), [Math.SQRT1_2, 0, -Math.SQRT1_2])).toBe(true); // up and forward
    expect(close(deflectAxis(motor, 90), [0, 0, -1])).toBe(true); // pure lift
    expect(close(deflectAxis(motor, 135), [-Math.SQRT1_2, 0, -Math.SQRT1_2])).toBe(true); // up and aft
  });

  it("uses the foil pressure point, deflection and loss for a foil fan", () => {
    const fan: Fan = { id: 0, output: "MAIN1", pos_frd_m: [-0.3, -0.37, -0.14], tilt_deg: 90, azimuth_deg: 0, spin: "CW", mirror_of: 1, curve_ref: "c" };
    const foil: Foil = {
      id: "left", fan_ids: [0, 2], deflection_deg: 45, per_fan_deflection_deg: { "2": 90 },
      pressure_points_frd_m: { "0": [-0.39, -0.37, -0.14] }, loss_at_90deg: 0.2,
    };
    const e = effectiveFan(fan, [foil]);
    expect(e.deflectionDeg).toBe(45);
    expect(e.pos).toEqual([-0.39, -0.37, -0.14]);
    expect(close(e.axis, [Math.SQRT1_2, 0, -Math.SQRT1_2])).toBe(true);
    expect(e.ctScale).toBeCloseTo(1 - 0.2 * 0.5, 9);
    const plain = effectiveFan({ ...fan, id: 8, tilt_deg: 0 }, [foil]);
    expect(plain.deflectionDeg).toBeNull();
    expect(close(plain.axis, [0, 0, -1])).toBe(true);
  });

  it("describes directions in words", () => {
    expect(describeAxis([0, 0, -1])).toBe("straight up");
    expect(describeAxis(deflectAxis(motor, 45))).toBe("up and forward (45 deg from vertical)");
    expect(describeAxis(deflectAxis(motor, 135))).toBe("up and aft (45 deg from vertical)");
  });

  it("forward/side tilt round-trips with tilt/azimuth and stays independent", () => {
    for (const [fwd, side] of [[0, 0], [10, 0], [0, -20], [25, 15], [-30, 40], [60, -60]]) {
      const { tilt, azimuth } = fwdSideToTiltAzimuth(fwd, side);
      const back = tiltAzimuthToFwdSide(tilt, azimuth);
      expect(back.fwd).toBeCloseTo(fwd, 6);
      expect(back.side).toBeCloseTo(side, 6);
      // the axis leans by tan(fwd) forward and tan(side) right per unit of down
      const a = tiltAzimuthToAxis(tilt, azimuth);
      expect(a[0] / -a[2]).toBeCloseTo(Math.tan((fwd * Math.PI) / 180), 6);
      expect(a[1] / -a[2]).toBeCloseTo(Math.tan((side * Math.PI) / 180), 6);
    }
    expect(fwdSideToTiltAzimuth(0, 20)).toEqual({ tilt: 20, azimuth: 90 });
    expect(fwdSideToTiltAzimuth(-15, 0)).toEqual({ tilt: 15, azimuth: 180 });
    expect(tiltAzimuthToFwdSide(90, 0).fwd).toBeCloseTo(90, 6);
  });

  it("negative and past-horizontal tilts", () => {
    expect(signedTilt(-30, 0)).toEqual({ tilt: 30, azimuth: 180 });
    expect(signedTilt(-30, 90)).toEqual({ tilt: 30, azimuth: 270 });
    expect(signedTilt(120, 0)).toEqual({ tilt: 120, azimuth: 0 });
    expect(signedTilt(200, 0).tilt).toBe(180);
    expect(clampTilt(-5)).toBe(0);
    // tilt 120 forward: thrust points forward and down
    const a = tiltAzimuthToAxis(120, 0);
    expect(a[0]).toBeGreaterThan(0);
    expect(a[2]).toBeGreaterThan(0);
    expect(tiltAzimuthToFwdSide(120, 0).fwd).toBeCloseTo(120, 6);
    expect(pastHorizontal(120)).toBe(true);
    expect(pastHorizontal(90)).toBe(false);
  });
});
