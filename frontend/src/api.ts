import type {
  BoardParamResult, BoardPushResult, BoardStatus, ControlConcept, FoilSheetRow, GazeboMode, GazeboStatus, Metrics, Scenario,
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
  launchGazebo: (scenario: Scenario, mode: GazeboMode) =>
    post<GazeboStatus>("/api/gazebo/launch", { scenario, mode }),
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
  /** Write one parameter (SYS_HITL for the HIL toggle) with the type the board reports. */
  boardParam: (name: string, value: number, port = "auto") =>
    post<BoardParamResult>("/api/board/param", { name, value, port }),
  /** Reboot the autopilot; the USB link is gone for a few seconds afterwards. */
  boardReboot: (port = "auto") => post<{ port: string; rebooted: boolean }>("/api/board/reboot", { port }),
};
