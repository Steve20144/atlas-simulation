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
