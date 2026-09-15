import type {
  BoardFlightResult, BoardLogResult, BoardParamResult, BoardParamValue, BoardPushResult, BoardStatus, ThrustSnapshot, ControlConcept, FoilSheetRow, GazeboMode, GazeboStatus, GazeboProbe, Metrics, ParamInfo, Scenario,
  SweepRequestBody, SweepResponse,
} from "./types";

async function request<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(url, init);
  if (!res.ok) {
    const text = await res.text().catch(() => "");
    let detail = text;
    try {
      detail = String((JSON.parse(text) as { detail?: unknown }).detail ?? text); // FastAPI error body
    } catch {
      // not JSON: keep the raw text
    }
    throw new Error(`${init?.method ?? "GET"} ${url} failed (${res.status}) ${detail}`.trim());
  }
  return (await res.json()) as T;
}

function post<T>(url: string, body: unknown): Promise<T> {
  return request<T>(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
}

export const api = {
  listScenarios: () =>
    request<{ scenarios: string[] }>("/api/scenarios").then((r) => r.scenarios),
  getScenario: (name: string) => request<Scenario>(`/api/scenarios/${encodeURIComponent(name)}`),
  saveScenario: (name: string, scenario: Scenario) =>
    post<unknown>(`/api/scenarios/${encodeURIComponent(name)}`, scenario),
  metrics: (scenario: Scenario, concept: ControlConcept, collective: number | null) =>
    post<Metrics>("/api/metrics", {
      scenario,
      concept,
      ...(collective === null ? {} : { collective }),
    }),
  paramsPreview: (scenario: Scenario, concept: ControlConcept) =>
    post<{ lines: string[] }>("/api/px4_params_preview", { scenario, concept }),
  exportParams: (scenario: Scenario, concept: ControlConcept) =>
    post<{ path: string }>("/api/export/params", { scenario, concept }),
  exportCsv: (rows: Record<string, unknown>[], stem: string) =>
    post<{ path: string }>("/api/export/csv", { rows, stem }),
  sweep: (body: SweepRequestBody) => post<SweepResponse>("/api/sweep", body),
  foilSheet: (scenario: Scenario) =>
    post<{ rows: FoilSheetRow[]; markdown: string }>("/api/foil_sheet", { scenario }),
  exportFoilSheet: (scenario: Scenario) =>
    post<{ csv_path: string; md_path: string }>("/api/export/foil_sheet", { scenario }),
  exportGazeboHitl: (scenario: Scenario) =>
    post<{ root: string; model_sdf: string; world: string; params: string; readme: string }>(
      "/api/export/gazebo_hitl",
      { scenario },
    ),
  /** Export the harness and start the WSL launcher for that mode; the console goes to exports/logs/. */
  launchGazebo: (scenario: Scenario, mode: GazeboMode, headless = false) =>
    post<GazeboStatus>("/api/gazebo/launch", { scenario, mode, headless }),
  takeoffGazebo: () => post<GazeboStatus & { note?: string }>("/api/gazebo/takeoff", {}),
  landGazebo: () => post<GazeboStatus & { note?: string }>("/api/gazebo/land", {}),
  gazeboProbe: () => request<GazeboProbe>("/api/gazebo/probe"),
  /** Copy the newest SITL ulog into exports/logs (sitl_<name>.ulg). */
  pullGazeboLog: () => post<{ path: string | null; name?: string; size?: number }>("/api/gazebo/log", {}),
  wslShutdown: () => post<GazeboStatus>("/api/gazebo/wsl_shutdown", {}),
  gazeboStatus: () => request<GazeboStatus>("/api/gazebo/status"),
  stopGazebo: () => post<GazeboStatus>("/api/gazebo/stop", {}),
  /** Disarm and put the model back where it spawned; HITL reboots the board if termination latched. */
  resetGazebo: () => post<GazeboStatus & { rebooted?: boolean; note?: string[] }>("/api/gazebo/reset", {}),
  exportGazebo: (scenario: Scenario) =>
    post<{ root: string; model_sdf: string; world_sdf: string; airframe: string; readme: string }>(
      "/api/export/gazebo",
      { scenario },
    ),
  /** Is a Pixhawk heartbeating on USB; answers at once when no Pixhawk-looking port exists. */
  boardStatus: (port = "auto") => request<BoardStatus>(`/api/board/status?port=${encodeURIComponent(port)}`),
  /** Write the previewed CA_* set to the board through its shell, save, read back; backup first. */
  boardPush: (scenario: Scenario, concept: ControlConcept, port = "auto") =>
    post<BoardPushResult>("/api/board/push", { scenario, concept, port }),
  /** Write one parameter (SYS_HITL 1 for HIL on) with the type the board reports. */
  boardParam: (name: string, value: number, port = "auto") =>
    post<BoardParamResult>("/api/board/param", { name, value, port }),
  /** Read one parameter's current value from the board (PX4 params editor). */
  boardReadParam: (name: string, port = "auto") =>
    request<BoardParamValue>(`/api/board/param?name=${encodeURIComponent(name)}&port=${encodeURIComponent(port)}`),
  /** Search the pinned PX4 tree's parameter catalogue by name or description. */
  paramSearch: (q: string, limit = 30) =>
    request<{ count: number; results: ParamInfo[] }>(`/api/px4/params?q=${encodeURIComponent(q)}&limit=${limit}`)
      .then((r) => r.results),
  paramInfo: (name: string) => request<ParamInfo>(`/api/px4/params/${encodeURIComponent(name)}`),
  /** HIL off that survives a reboot: SYS_HITL 0 plus SYS_AUTOSTART and the rest of the HITL set
   * restored from the flight backup (the HIL airframe re-enables SYS_HITL at boot otherwise). */
  boardFlight: (scenario: Scenario, port = "auto") =>
    post<BoardFlightResult>("/api/board/flight", { scenario, port }),
  /** Reboot the autopilot; the USB link is gone for a few seconds afterwards. */
  boardReboot: (port = "auto") => post<{ port: string; rebooted: boolean }>("/api/board/reboot", { port }),
  /** Download the newest flight log from the board's SD card into exports/logs/ (slow over USB). */
  boardPullLog: (port = "auto") => post<BoardLogResult>("/api/board/pull_log", { port }),
  /** Test thrust: live stick -> throttle -> thrust setpoint -> ESC pulse widths from the board. */
  thrustFeedStart: (port = "auto") => post<ThrustSnapshot>("/api/board/thrust_feed/start", { port }),
  thrustFeed: (seconds = 30) => request<ThrustSnapshot>(`/api/board/thrust_feed?seconds=${seconds}`),
  thrustFeedStop: () => post<{ stopped: boolean }>("/api/board/thrust_feed/stop", {}),
};
