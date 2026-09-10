import { create } from "zustand";
import type { Scenario, Vec3 } from "./types";

const zero3 = (): Vec3 => [0, 0, 0];

/** Empty scenario used before one is loaded from the backend. */
export function emptyScenario(): Scenario {
  return {
    meta: { name: "untitled", created: "", px4_version: "1.17.0" },
    frame: { cad_forward_axis: "+X", cad_up_axis: "+Z", cad_units: "mm" },
    mass: {
      total_kg: 0,
      cg_frd_m: zero3(),
      inertia_frd_kgm2: [zero3(), zero3(), zero3()],
      cad_reported: { mass_kg: 0, cg_m: zero3() },
      bodies: [],
    },
    fans: [],
    fan_curves: {},
    control: { concept: "stock", blend: 0, ca_method: 0, px4_params_override: {} },
    rig: {
      enabled: false,
      lock_position: true,
      lock_roll: true,
      lock_pitch: false,
      lock_yaw: false,
      attitude_offset_deg: zero3(),
    },
    environment: { air_density: 1.225, wind_ned_mps: zero3(), gravity: 9.80665 },
    outputs: {
      params: true,
      csv: true,
      report: true,
      plots: true,
      sdf: false,
      angle_sheet: true,
    },
  };
}

export interface ScenarioState {
  scenario: Scenario;
  setScenario: (scenario: Scenario) => void;
  updateScenario: (patch: Partial<Scenario>) => void;
  reset: () => void;
}

export const useScenarioStore = create<ScenarioState>((set) => ({
  scenario: emptyScenario(),
  setScenario: (scenario) => set({ scenario }),
  updateScenario: (patch) => set((s) => ({ scenario: { ...s.scenario, ...patch } })),
  reset: () => set({ scenario: emptyScenario() }),
}));
