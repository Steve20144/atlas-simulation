import type { ControlConcept, Metrics, Scenario } from "./types";

async function request<T>(url: string, init?: RequestInit): Promise<T> {
  const res = await fetch(url, init);
  if (!res.ok) {
    const text = await res.text().catch(() => "");
    throw new Error(`${init?.method ?? "GET"} ${url} failed (${res.status}) ${text}`.trim());
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
};
