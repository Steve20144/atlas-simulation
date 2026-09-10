import { vi } from "vitest";
import { emptyScenario } from "../store";
import type { Fan, Metrics, Scenario } from "../types";

/** Ten-fan scenario shaped like scenarios/baseline_dihedral30.json (pairs 0/1 .. 6/7, 8 and 9 single). */
export function tenFanScenario(): Scenario {
  const base = emptyScenario();
  const fans: Fan[] = [];
  for (let i = 0; i < 8; i++) {
    const left = i % 2 === 0;
    fans.push({
      id: i,
      output: `MAIN${i + 1}`,
      pos_frd_m: [-0.3 + 0.05 * Math.floor(i / 2), left ? -0.37 : 0.37, -0.14],
      tilt_deg: 30,
      azimuth_deg: left ? 90 : 270,
      spin: "CW",
      mirror_of: left ? i + 1 : i - 1,
      curve_ref: "xfly80_3280",
      km: 0,
    });
  }
  for (const i of [8, 9]) {
    fans.push({
      id: i,
      output: `MAIN${i + 1}`,
      pos_frd_m: [0.45 + 0.11 * (i - 8), 0, 0.08],
      tilt_deg: 0,
      azimuth_deg: 0,
      spin: "CW",
      mirror_of: null,
      curve_ref: "xfly80_3280",
      km: 0,
    });
  }
  return {
    ...base,
    meta: { ...base.meta, name: "fixture10" },
    mass: { ...base.mass, total_kg: 12, estimated: true },
    fans,
    fan_curves: {
      xfly80_3280: {
        cells: 6,
        points: [
          { cmd: 0, thrust_N: 0, power_W: 0 },
          { cmd: 1, thrust_N: 33.3, power_W: 2450 },
        ],
        lag_s: 0.15,
        max_continuous_A: 100,
        notes: "",
        estimated: true,
      },
    },
  };
}

export function sampleMetrics(): Metrics {
  const axis = {
    plus: 1,
    minus: 1,
    unit: "N m",
    plus_per_W: 0.001,
    minus_per_W: 0.001,
    plus_attainable: true,
    minus_attainable: true,
    badge: "stock_px4",
  };
  const fa = { ...axis, unit: "N", badge: "needs_fully_actuated_controller" };
  const axes = ["roll", "pitch", "yaw", "Fx", "Fy", "Fz"];
  return {
    scenario_name: "fixture10",
    concept: "fully_actuated",
    controlled_axes: axes,
    collective: 0.4,
    hover: { u: new Array(10).fill(0.4), thrust_N: new Array(10).fill(11.8), power_W: 1500, headroom: 0.6 },
    authority: { roll: axis, pitch: axis, yaw: axis, Fx: fa, Fy: fa, Fz: { ...fa, badge: "stock_px4" } },
    coupling: {
      axes,
      leakage_fraction: axes.map((_, i) => axes.map((_, j) => (i === j ? 1 : 0))),
      max_offaxis_fraction: 0,
      badge: "needs_fully_actuated_controller",
    },
    conditioning: { axes, singular_values: [1, 0.5, 0.2, 0.1], rank: 4, null_space_dim: 6, condition_number: 10 },
    score: { value: 0.72, weights: { authority_balance: 0.5, decoupling: 0.5 } },
    badges: { hover: "needs_fully_actuated_controller" },
    estimated: true,
    estimated_sources: ["mass", "fan_curve:xfly80_3280"],
    notes: [],
  };
}

export const PARAM_LINES = ["1\t1\tCA_ROTOR0_PX\t-0.300000\t9", "1\t1\tCA_ROTOR0_PY\t-0.370000\t9"];

type FetchCall = { url: string; method: string; body: unknown };

/**
 * Install a fetch mock answering the M4/M5 API contract. Returns the recorded calls so tests can
 * inspect the scenario that was posted.
 */
export function installFetchMock(scenario: Scenario = tenFanScenario()) {
  const calls: FetchCall[] = [];
  const respond = (data: unknown) =>
    Promise.resolve({ ok: true, status: 200, json: async () => data, text: async () => "" } as Response);
  const fetchMock = vi.fn((input: RequestInfo | URL, init?: RequestInit) => {
    const url = String(input);
    const method = init?.method ?? "GET";
    const body = init?.body ? JSON.parse(String(init.body)) : undefined;
    calls.push({ url, method, body });
    if (url === "/api/metrics") return respond(sampleMetrics());
    if (url === "/api/px4_params_preview") return respond({ lines: PARAM_LINES });
    if (url === "/api/scenarios") return respond({ scenarios: ["baseline_dihedral30", "fixture10"] });
    if (url.startsWith("/api/scenarios/") && method === "GET") return respond(scenario);
    if (url.startsWith("/api/scenarios/") && method === "POST") return respond({ ok: true });
    return Promise.resolve({ ok: false, status: 404, json: async () => ({}), text: async () => "" } as Response);
  });
  vi.stubGlobal("fetch", fetchMock);
  return { calls, fetchMock };
}
