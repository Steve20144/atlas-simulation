import { describe, expect, it } from "vitest";
import { deflectAxis, describeAxis, effectiveFan, tiltAzimuthToAxis } from "./geometry";
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
});
