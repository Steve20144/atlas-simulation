import { create } from "zustand";
import { api } from "./api";
import { mirrorAzimuth } from "./geometry";
import { DIHEDRAL_SCENARIO_NAME, presetOmni, presetVertical, type PresetId } from "./presets";
import type {
  Coanda,
  ControlConcept,
  Fan,
  Foil,
  MetricGroup,
  Metrics,
  Scenario,
  SweepCandidate,
  Vec3,
} from "./types";

const zero3 = (): Vec3 => [0, 0, 0];

/** Debounce for POST /api/metrics and /api/px4_params_preview after a scenario edit. */
export const REFRESH_DEBOUNCE_MS = 50;

/** Empty scenario used before one is loaded from the backend. */
export function emptyScenario(): Scenario {
  return {
    meta: { name: "untitled", created: "", px4_version: "1.17.0" },
    frame: { cad_forward_axis: "+X", cad_up_axis: "+Z", cad_units: "mm" },
    mass: {
      total_kg: 0,
      cg_frd_m: zero3(),
      inertia_frd_kgm2: [zero3(), zero3(), zero3()],
      cad_reported: null,
      bodies: [],
    },
    fans: [],
    fan_curves: {},
    control: { concept: "stock", blend: 0, ca_method: 2, px4_params_override: {} },
    rig: {
      enabled: false,
      lock_position: true,
      lock_roll: true,
      lock_pitch: false,
      lock_yaw: false,
      attitude_offset_deg: zero3(),
    },
    environment: { air_density: 1.225, wind_ned_mps: zero3(), gravity: 9.80665 },
    outputs: { params: true, csv: true, report: true, plots: true, sdf: false, angle_sheet: true },
    foils: [],
  };
}

/** Mirror lock defaults to on for every fan that has a partner (PLAN.md section 3). */
function defaultMirrorLock(scenario: Scenario): Record<number, boolean> {
  const lock: Record<number, boolean> = {};
  for (const f of scenario.fans) lock[f.id] = f.mirror_of !== null && f.mirror_of !== undefined;
  return lock;
}

export type FanPatch = Partial<Pick<Fan, "tilt_deg" | "azimuth_deg" | "output" | "spin">>;

export interface TiltlabState {
  scenario: Scenario;
  scenarioNames: string[];
  concept: ControlConcept;
  /** Collective command 0..1; null means "hover" (backend solves for hover). */
  collective: number | null;
  metrics: Metrics | null;
  paramsLines: string[];
  loading: boolean;
  error: string | null;
  mirrorLock: Record<number, boolean>;
  visibleGroups: Record<MetricGroup, boolean>;
  /** When on, changing one foil's deflection changes every foil (a single foil angle). */
  foilLinked: boolean;

  setScenario: (scenario: Scenario) => void;
  setFoilLinked: (on: boolean) => void;
  /** Set a foil's deflection (all its fans); with foilLinked every foil follows. */
  setFoilDeflection: (foilId: string, deg: number) => void;
  /** Set one fan's deflection inside a segmented foil (null clears the override). */
  setFoilFanDeflection: (foilId: string, fanId: number, deg: number | null) => void;
  setFoilLoss: (loss: number) => void;
  /** Patch the Coanda surface model on every foil (radius, jet thickness, losses, enabled). */
  setCoanda: (patch: Partial<Coanda>) => void;
  /** Load a sweep candidate: foil deflections or fan tilts depending on its variable. */
  applySweepCandidate: (c: SweepCandidate) => void;
  updateFan: (id: number, patch: FanPatch) => void;
  setMirrorLock: (id: number, on: boolean) => void;
  setConcept: (concept: ControlConcept) => void;
  setCollective: (collective: number | null) => void;
  /** Set tilt and azimuth of every fan by index (from a sweep candidate) and refresh metrics. */
  applyFanAngles: (tilts_deg: number[], azimuths_deg: number[]) => void;
  toggleGroup: (group: MetricGroup) => void;
  applyPreset: (preset: PresetId) => Promise<void>;
  loadScenarioNames: () => Promise<void>;
  loadScenario: (name: string) => Promise<void>;
  saveScenario: () => Promise<void>;
  refresh: () => Promise<void>;
  reset: () => void;
}

let refreshTimer: ReturnType<typeof setTimeout> | null = null;
let refreshSeq = 0;

export const useTiltlabStore = create<TiltlabState>((set, get) => {
  const schedule = () => {
    if (refreshTimer) clearTimeout(refreshTimer);
    refreshTimer = setTimeout(() => {
      refreshTimer = null;
      void get().refresh();
    }, REFRESH_DEBOUNCE_MS);
  };

  const setScenarioAndRefresh = (scenario: Scenario) => {
    set({ scenario });
    schedule();
  };

  return {
    scenario: emptyScenario(),
    scenarioNames: [],
    concept: "stock",
    collective: null,
    metrics: null,
    paramsLines: [],
    loading: false,
    error: null,
    mirrorLock: {},
    visibleGroups: { hover: true, authority: true, control: true, coupling: true, conditioning: true, composite: true },
    foilLinked: true,

    setFoilLinked: (on) => set({ foilLinked: on }),

    setFoilDeflection: (foilId, deg) => {
      const { scenario, foilLinked } = get();
      const foils: Foil[] = (scenario.foils ?? []).map((f) =>
        f.id === foilId || foilLinked ? { ...f, deflection_deg: deg, per_fan_deflection_deg: {} } : f,
      );
      setScenarioAndRefresh({ ...scenario, foils });
    },

    setFoilFanDeflection: (foilId, fanId, deg) => {
      const { scenario } = get();
      const foils: Foil[] = (scenario.foils ?? []).map((f) => {
        if (f.id !== foilId) return f;
        const per = { ...f.per_fan_deflection_deg };
        if (deg === null) delete per[String(fanId)];
        else per[String(fanId)] = deg;
        return { ...f, per_fan_deflection_deg: per };
      });
      setScenarioAndRefresh({ ...scenario, foils });
    },

    setFoilLoss: (loss) => {
      const { scenario } = get();
      const foils: Foil[] = (scenario.foils ?? []).map((f) => ({ ...f, loss_at_90deg: loss }));
      setScenarioAndRefresh({ ...scenario, foils });
    },

    setCoanda: (patch) => {
      const { scenario } = get();
      const base: Coanda = {
        enabled: true, radius_m: 0.25, jet_thickness_m: 0.08, theta0_deg: 245, k: 1.64,
        loss_per_90deg: 0.1, separated_loss: 0.3, estimated: true,
      };
      const foils: Foil[] = (scenario.foils ?? []).map((f) => ({ ...f, coanda: { ...base, ...(f.coanda ?? {}), ...patch } }));
      setScenarioAndRefresh({ ...scenario, foils });
    },

    applySweepCandidate: (c) => {
      const { scenario } = get();
      if (c.variable === "foil" && c.deflections_deg) {
        const d = c.deflections_deg;
        const foils: Foil[] = (scenario.foils ?? []).map((f) => {
          const own = f.fan_ids.filter((i) => String(i) in d);
          const vals = new Set(own.map((i) => d[String(i)]));
          if (own.length === f.fan_ids.length && vals.size === 1) {
            return { ...f, deflection_deg: [...vals][0], per_fan_deflection_deg: {} };
          }
          const per = { ...f.per_fan_deflection_deg };
          for (const i of own) per[String(i)] = d[String(i)];
          return { ...f, per_fan_deflection_deg: per };
        });
        setScenarioAndRefresh({ ...scenario, foils });
      } else if (c.tilts_deg && c.azimuths_deg) {
        get().applyFanAngles(c.tilts_deg, c.azimuths_deg);
      }
    },

    setScenario: (scenario) => {
      set({ scenario, concept: scenario.control.concept, mirrorLock: defaultMirrorLock(scenario) });
      schedule();
    },

    updateFan: (id, patch) => {
      const { scenario, mirrorLock } = get();
      const fan = scenario.fans.find((f) => f.id === id);
      if (!fan) return;
      const partnerId = mirrorLock[id] ? fan.mirror_of : null;
      const partnerPatch: FanPatch = {};
      if (patch.tilt_deg !== undefined) partnerPatch.tilt_deg = patch.tilt_deg;
      if (patch.azimuth_deg !== undefined) partnerPatch.azimuth_deg = mirrorAzimuth(patch.azimuth_deg);
      const fans = scenario.fans.map((f) => {
        if (f.id === id) return { ...f, ...patch };
        if (partnerId !== null && f.id === partnerId) return { ...f, ...partnerPatch };
        return f;
      });
      setScenarioAndRefresh({ ...scenario, fans });
    },

    setMirrorLock: (id, on) => set((s) => ({ mirrorLock: { ...s.mirrorLock, [id]: on } })),

    setConcept: (concept) => {
      const { scenario } = get();
      set({ concept });
      setScenarioAndRefresh({ ...scenario, control: { ...scenario.control, concept } });
    },

    setCollective: (collective) => {
      set({ collective });
      schedule();
    },

    applyFanAngles: (tilts_deg, azimuths_deg) => {
      const { scenario } = get();
      const fans = scenario.fans.map((f, i) =>
        i < tilts_deg.length ? { ...f, tilt_deg: tilts_deg[i], azimuth_deg: azimuths_deg[i] ?? f.azimuth_deg } : f,
      );
      setScenarioAndRefresh({ ...scenario, fans });
    },

    toggleGroup: (group) =>
      set((s) => ({ visibleGroups: { ...s.visibleGroups, [group]: !s.visibleGroups[group] } })),

    applyPreset: async (preset) => {
      const { scenario, loadScenario } = get();
      if (preset === "dihedral30") return loadScenario(DIHEDRAL_SCENARIO_NAME);
      setScenarioAndRefresh(preset === "vertical" ? presetVertical(scenario) : presetOmni(scenario));
    },

    loadScenarioNames: async () => {
      try {
        set({ scenarioNames: await api.listScenarios(), error: null });
      } catch (e) {
        set({ error: (e as Error).message });
      }
    },

    loadScenario: async (name) => {
      set({ loading: true });
      try {
        get().setScenario(await api.getScenario(name));
        set({ error: null });
      } catch (e) {
        set({ error: (e as Error).message, loading: false });
      }
    },

    saveScenario: async () => {
      const { scenario } = get();
      try {
        await api.saveScenario(scenario.meta.name, scenario);
        set({ error: null });
        await get().loadScenarioNames();
      } catch (e) {
        set({ error: (e as Error).message });
      }
    },

    refresh: async () => {
      const seq = ++refreshSeq;
      const { scenario, concept, collective } = get();
      if (scenario.fans.length === 0) return;
      set({ loading: true });
      try {
        const [metrics, preview] = await Promise.all([
          api.metrics(scenario, concept, collective),
          api.paramsPreview(scenario, concept),
        ]);
        if (seq !== refreshSeq) return; // a newer edit superseded this response
        set({ metrics, paramsLines: preview.lines, loading: false, error: null });
      } catch (e) {
        if (seq !== refreshSeq) return;
        set({ loading: false, error: (e as Error).message });
      }
    },

    reset: () => {
      if (refreshTimer) clearTimeout(refreshTimer);
      refreshTimer = null;
      set({
        scenario: emptyScenario(),
        concept: "stock",
        collective: null,
        metrics: null,
        paramsLines: [],
        loading: false,
        error: null,
        mirrorLock: {},
      });
    },
  };
});
