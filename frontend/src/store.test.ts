import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { REFRESH_DEBOUNCE_MS, useTiltlabStore } from "./store";
import { installFetchMock, tenFanScenario } from "./test/fixtures";
import type { Scenario } from "./types";

async function flush() {
  await vi.advanceTimersByTimeAsync(REFRESH_DEBOUNCE_MS + 5);
}

describe("tiltlab store", () => {
  let calls: ReturnType<typeof installFetchMock>["calls"];

  beforeEach(() => {
    vi.useFakeTimers();
    calls = installFetchMock().calls;
    useTiltlabStore.getState().reset();
    useTiltlabStore.getState().setScenario(tenFanScenario());
  });

  afterEach(() => {
    useTiltlabStore.getState().reset();
    vi.unstubAllGlobals();
    vi.useRealTimers();
  });

  it("posts metrics and params preview after a debounced scenario load", async () => {
    expect(calls).toHaveLength(0);
    await flush();
    const urls = calls.map((c) => c.url).sort();
    expect(urls).toEqual(["/api/metrics", "/api/px4_params_preview"]);
    const s = useTiltlabStore.getState();
    expect(s.metrics?.hover.u).toHaveLength(10);
    expect(s.paramsLines).toHaveLength(2);
    expect(s.loading).toBe(false);
  });

  it("tilt edit triggers a metrics request carrying the changed scenario", async () => {
    await flush();
    calls.length = 0;
    useTiltlabStore.getState().updateFan(8, { tilt_deg: 12 });
    useTiltlabStore.getState().updateFan(8, { tilt_deg: 13 }); // coalesced by the debounce
    expect(calls).toHaveLength(0);
    await flush();
    const metricsCalls = calls.filter((c) => c.url === "/api/metrics");
    expect(metricsCalls).toHaveLength(1);
    const body = metricsCalls[0].body as { scenario: Scenario; concept: string };
    expect(body.concept).toBe("stock");
    expect(body.scenario.fans[8].tilt_deg).toBe(13);
    expect(body.scenario.fans[9].tilt_deg).toBe(0);
  });

  it("mirror lock mirrors azimuth and copies tilt to the partner", () => {
    const { updateFan } = useTiltlabStore.getState();
    expect(useTiltlabStore.getState().mirrorLock[0]).toBe(true);
    updateFan(0, { tilt_deg: 40, azimuth_deg: 80 });
    let fans = useTiltlabStore.getState().scenario.fans;
    expect(fans[0]).toMatchObject({ tilt_deg: 40, azimuth_deg: 80 });
    expect(fans[1]).toMatchObject({ tilt_deg: 40, azimuth_deg: 280 });
    expect(fans[2].azimuth_deg).toBe(90);

    useTiltlabStore.getState().setMirrorLock(0, false);
    updateFan(0, { azimuth_deg: 10 });
    fans = useTiltlabStore.getState().scenario.fans;
    expect(fans[0].azimuth_deg).toBe(10);
    expect(fans[1].azimuth_deg).toBe(280);
  });

  it("collective is sent only when set, and concept switch updates scenario.control", async () => {
    useTiltlabStore.getState().setConcept("fully_actuated");
    useTiltlabStore.getState().setCollective(0.7);
    await flush();
    const body = calls.find((c) => c.url === "/api/metrics")!.body as Record<string, unknown>;
    expect(body.collective).toBe(0.7);
    expect(body.concept).toBe("fully_actuated");
    expect((body.scenario as Scenario).control.concept).toBe("fully_actuated");
    calls.length = 0;
    useTiltlabStore.getState().setCollective(null);
    await flush();
    const body2 = calls.find((c) => c.url === "/api/metrics")!.body as Record<string, unknown>;
    expect("collective" in body2).toBe(false);
  });

  it("presets change all 10 fans", async () => {
    await useTiltlabStore.getState().applyPreset("vertical");
    let fans = useTiltlabStore.getState().scenario.fans;
    expect(fans).toHaveLength(10);
    expect(fans.every((f) => f.tilt_deg === 0)).toBe(true);

    await useTiltlabStore.getState().applyPreset("omni");
    fans = useTiltlabStore.getState().scenario.fans;
    expect(fans.every((f) => f.tilt_deg === 45)).toBe(true);
    expect(fans.map((f) => f.azimuth_deg)).toEqual([0, 90, 180, 270, 0, 90, 180, 270, 0, 90]);

    await useTiltlabStore.getState().applyPreset("dihedral30");
    expect(calls.some((c) => c.url === "/api/scenarios/baseline_dihedral30" && c.method === "GET")).toBe(true);
    fans = useTiltlabStore.getState().scenario.fans;
    expect(fans.slice(0, 8).every((f) => f.tilt_deg === 30)).toBe(true);
  });

  it("applying a foil sweep row sets the foil deflections and the nose fans' sideways tilt", () => {
    const base = useTiltlabStore.getState().scenario;
    const scenario: Scenario = {
      ...base,
      foils: [
        { id: "left", fan_ids: [0, 2, 4, 6], deflection_deg: 45, per_fan_deflection_deg: {} },
        { id: "right", fan_ids: [1, 3, 5, 7], deflection_deg: 45, per_fan_deflection_deg: {} },
      ] as Scenario["foils"],
    };
    useTiltlabStore.getState().setScenario(scenario);
    useTiltlabStore.getState().applySweepCandidate({
      variable: "foil",
      deflections_deg: { "0": 135, "1": 135, "2": 45, "3": 45, "4": 45, "5": 45, "6": 45, "7": 45 },
      pair_tilts_deg: [135, 45, 45, 45],
      centreline_tilt_deg: 20,
      nose_tilts_deg: [20, -20],
      nose_angles_deg: { "9": [20, 90], "8": [20, 270] },
      hover_pitch_deg: 10,
      power_W: 1, headroom: 0.5, roll_Nm: 1, pitch_Nm: 1, yaw_Nm: 1, fz_up_N: 1, yaw_Nm_per_kW: 1,
      coupling_max: 0, condition_number: 1, roll_acc: 1, pitch_acc: 1, yaw_acc: 1, linear_frac: 1,
      surge_leak: 0, control_score: 1, weakest_axis: null, score: 1, estimated: false, feasible: true, reasons: [],
    });
    const sc = useTiltlabStore.getState().scenario;
    const fan = (id: number) => sc.fans.find((f) => f.id === id)!;
    expect(fan(9).tilt_deg).toBe(20);
    expect(fan(9).azimuth_deg).toBe(90);
    expect(fan(8).tilt_deg).toBe(20);
    expect(fan(8).azimuth_deg).toBe(270);
    expect(fan(0).tilt_deg).toBe(base.fans.find((f) => f.id === 0)!.tilt_deg);
    expect(sc.foils?.[0].per_fan_deflection_deg).toEqual({ "0": 135, "2": 45, "4": 45, "6": 45 });
    expect(sc.frame.hover_pitch_deg).toBe(10);
  });

  it("records backend errors without crashing", async () => {
    vi.stubGlobal(
      "fetch",
      vi.fn(() => Promise.resolve({ ok: false, status: 500, text: async () => "boom", json: async () => ({}) })),
    );
    useTiltlabStore.getState().updateFan(0, { tilt_deg: 1 });
    await flush();
    expect(useTiltlabStore.getState().error).toContain("500");
    expect(useTiltlabStore.getState().loading).toBe(false);
  });
});
