import { create } from "zustand";
import { api } from "./api";
import { mirrorAzimuth } from "./geometry";
import { DIHEDRAL_SCENARIO_NAME, presetOmni, presetVertical, type PresetId } from "./presets";
import type {
  BoardPushResult,
  BoardStatus,
  GazeboMode,
  GazeboStatus,
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

/** How long the Pixhawk's USB link takes to come back after a reboot before re-checking. */
export let REBOOT_RECHECK_MS = 8000;
export function setRebootRecheckMs(ms: number): void {
  REBOOT_RECHECK_MS = ms;
}

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

/** Which panels and viewer layers are shown; toggled from the side rail. */
export type ViewFlag = "geometry" | "metrics" | "cad" | "flow";

/** Pixhawk over USB: last status check, last push, and the port the user picked ("auto"). */
export interface BoardState {
  status: BoardStatus | null;
  checking: boolean;
  pushing: boolean;
  result: BoardPushResult | null;
  message: string;
  port: string;
}

const idleBoard = (): BoardState => ({
  status: null, checking: false, pushing: false, result: null, message: "", port: "auto",
});

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
  /** Gazebo session started from the app (SITL or HITL); message is the last outcome shown. */
  gazebo: GazeboStatus & { message: string };
  view: Record<ViewFlag, boolean>;
  board: BoardState;

  toggleView: (flag: ViewFlag) => void;
  setBoardPort: (port: string) => void;
  /** GET /api/board/status: the status button in the top bar and the board panel. */
  checkBoard: () => Promise<void>;
  /** POST /api/board/push for the current scenario and concept (the previewed lines). */
  pushBoard: () => Promise<void>;
  /** Write SYS_HITL (0 off, 1 HITL) and refresh the status; PX4 reads it at boot only. */
  setBoardHitl: (on: boolean) => Promise<void>;
  /** Reboot the autopilot and re-check after the USB link is back. */
  rebootBoard: () => Promise<void>;
  setScenario: (scenario: Scenario) => void;
  setFoilLinked: (on: boolean) => void;
  /** Hover attitude of the airframe, nose-up degrees. PX4's body frame is this hover frame, so the
   * exported geometry, the hover trim and every authority number depend on it. Refreshes metrics. */
  setHoverPitch: (deg: number) => void;
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
  launchGazebo: (mode: GazeboMode) => Promise<void>;
  pollGazebo: () => Promise<void>;
  stopGazebo: () => Promise<void>;
  resetGazebo: () => Promise<void>;
  reset: () => void;
}

const idleGazebo = (): GazeboStatus & { message: string } => ({
  available: false, running: false, mode: null, harness: null, command: null, log: null, tail: [],
  returncode: null, message: "",
});

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
    gazebo: idleGazebo(),
    view: { geometry: true, metrics: true, cad: true, flow: true },
    board: idleBoard(),
    toggleView: (flag) => set({ view: { ...get().view, [flag]: !get().view[flag] } }),
    setBoardPort: (port) => set({ board: { ...get().board, port } }),
    checkBoard: async () => {
      set({ board: { ...get().board, checking: true } });
      try {
        const status = await api.boardStatus(get().board.port);
        set({ board: { ...get().board, status, checking: false, message: status.message } });
      } catch (e) {
        set({ board: { ...get().board, status: null, checking: false, message: (e as Error).message } });
      }
    },
    pushBoard: async () => {
      set({ board: { ...get().board, pushing: true, result: null } });
      try {
        const result = await api.boardPush(get().scenario, get().concept, get().board.port);
        const ok = result.mismatches.length === 0;
        const message = ok
          ? `${result.verified} of ${result.sent} parameters verified on ${result.port}, ${result.changed.length} changed`
          : `${result.mismatches.length} of ${result.sent} parameters did not read back`;
        set({ board: { ...get().board, result, pushing: false, message } });
      } catch (e) {
        set({ board: { ...get().board, pushing: false, message: (e as Error).message } });
      }
    },

    setBoardHitl: async (on) => {
      set({ board: { ...get().board, pushing: true } });
      try {
        const r = await api.boardParam("SYS_HITL", on ? 1 : 0, get().board.port);
        const status = get().board.status;
        const message = r.verified
          ? `SYS_HITL ${r.before} -> ${r.after} saved; reboot the board for it to take effect`
          : `SYS_HITL did not read back (board has ${r.after})`;
        set({
          board: {
            ...get().board,
            pushing: false,
            message,
            status: status ? { ...status, sys_hitl: r.after === null ? null : Math.round(r.after) } : status,
          },
        });
      } catch (e) {
        set({ board: { ...get().board, pushing: false, message: (e as Error).message } });
      }
    },
    rebootBoard: async () => {
      set({ board: { ...get().board, pushing: true, message: "rebooting" } });
      try {
        await api.boardReboot(get().board.port);
        set({ board: { ...get().board, pushing: false, status: null, message: "reboot sent; checking again in 8 s" } });
        await new Promise((r) => setTimeout(r, REBOOT_RECHECK_MS));
        await get().checkBoard();
      } catch (e) {
        set({ board: { ...get().board, pushing: false, message: (e as Error).message } });
      }
    },
    launchGazebo: async (mode) => {
      try {
        const st = await api.launchGazebo(get().scenario, mode);
        const message = st.dry_run
          ? `dry run, would run: ${st.command ?? ""}`
          : `${mode.toUpperCase()} launching; console in ${st.log ?? "exports/logs/"}`;
        set({ gazebo: { ...st, message } });
      } catch (e) {
        set({ gazebo: { ...get().gazebo, message: (e as Error).message } });
      }
    },
    pollGazebo: async () => {
      try {
        const st = await api.gazeboStatus();
        set({ gazebo: { ...st, message: get().gazebo.message } });
      } catch (e) {
        set({ gazebo: { ...get().gazebo, message: (e as Error).message } });
      }
    },
    resetGazebo: async () => {
      try {
        const st = await api.resetGazebo();
        const message = st.rebooted
          ? "flight termination was latched: board rebooted and the sim relaunched"
          : `reset: ${(st.note ?? []).join(" | ") || "poses back, disarmed"}`;
        set({ gazebo: { ...st, message } });
      } catch (e) {
        set({ gazebo: { ...get().gazebo, message: (e as Error).message } });
      }
    },
    stopGazebo: async () => {
      try {
        const st = await api.stopGazebo();
        set({ gazebo: { ...st, message: "stopped" } });
      } catch (e) {
        set({ gazebo: { ...get().gazebo, message: (e as Error).message } });
      }
    },

    setFoilLinked: (on) => set({ foilLinked: on }),
    setHoverPitch: (deg) => {
      const clamped = Math.max(-90, Math.min(90, deg));
      const { scenario } = get();
      setScenarioAndRefresh({ ...scenario, frame: { ...scenario.frame, hover_pitch_deg: clamped } });
    },

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
      const base = get().scenario;
      const scenario = c.hover_pitch_deg === undefined ? base : { ...base, frame: { ...base.frame, hover_pitch_deg: c.hover_pitch_deg } };
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
        // nose fans swept sideways: take the tilt/azimuth the sweep applied
        const nose = c.nose_angles_deg ?? {};
        const fans = scenario.fans.map((f) => {
          const a = nose[String(f.id)];
          return a ? { ...f, tilt_deg: a[0], azimuth_deg: a[1] } : f;
        });
        setScenarioAndRefresh({ ...scenario, foils, fans });
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
        gazebo: idleGazebo(),
        board: idleBoard(),
        view: { geometry: true, metrics: true, cad: true, flow: true },
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
