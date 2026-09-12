import { useEffect } from "react";
import { useTiltlabStore } from "../store";
import type { ControlConcept } from "../types";
import BoardPill from "./BoardPill";

const CONCEPTS: { id: ControlConcept; label: string; title: string }[] = [
  {
    id: "stock",
    label: "Stock",
    title:
      "PX4 as it ships: the controller asks the fans only for roll, pitch, yaw and vertical thrust and moves by tilting the aircraft. Fans that push forward or sideways at level hover fight the allocator. Achievable with today's firmware.",
  },
  {
    id: "fully_actuated",
    label: "Fully actuated",
    title:
      "A modified controller that commands all six axes (three torques plus forward, sideways and vertical force), so the aircraft can translate while level. Needs a patched PX4 or an offboard controller; shows what the airframe could do, not what it does now.",
  },
];

/** Brand mark: four fan discs in a ring. */
function Mark() {
  return (
    <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7">
      <circle cx="7" cy="7" r="3.2" />
      <circle cx="17" cy="7" r="3.2" />
      <circle cx="7" cy="17" r="3.2" />
      <circle cx="17" cy="17" r="3.2" />
      <path d="M10 10l4 4M14 10l-4 4" strokeWidth="1.2" />
    </svg>
  );
}

/** Top bar: brand and scenario on the left, concept tabs centred, board status and activity right. */
export default function TopBar() {
  const scenarioName = useTiltlabStore((s) => s.scenario.meta.name);
  const scenarioNames = useTiltlabStore((s) => s.scenarioNames);
  const concept = useTiltlabStore((s) => s.concept);
  const loading = useTiltlabStore((s) => s.loading);
  const error = useTiltlabStore((s) => s.error);
  const { loadScenarioNames, loadScenario, saveScenario, setConcept } = useTiltlabStore.getState();

  useEffect(() => {
    void loadScenarioNames();
  }, [loadScenarioNames]);

  return (
    <header
      className="grid grid-cols-[1fr_auto_1fr] items-center gap-4 px-3 py-2"
      style={{ borderBottom: "1px solid var(--ui-line-soft)" }}
    >
      <div className="flex items-center gap-3">
        <h1 className="flex items-center gap-2 text-sm">
          <Mark />
          tiltlab
        </h1>
        <span className="h-4 w-px" style={{ background: "var(--ui-line)" }} />
        <label className="flex items-center gap-2">
          <span className="ui-label">Scenario</span>
          <select
            aria-label="Scenario"
            className="max-w-[220px]"
            value={scenarioNames.includes(scenarioName) ? scenarioName : ""}
            onChange={(e) => e.target.value && void loadScenario(e.target.value)}
          >
            {!scenarioNames.includes(scenarioName) && <option value="">{scenarioName}</option>}
            {scenarioNames.map((n) => (
              <option key={n} value={n}>
                {n}
              </option>
            ))}
          </select>
          <button className="ui-btn" onClick={() => void saveScenario()} title="POST /api/scenarios/{name}">
            Save
          </button>
        </label>
      </div>

      <div className="ui-tabs" role="group" aria-label="Concept">
        {CONCEPTS.map((c) => (
          <button
            key={c.id}
            className="ui-tab"
            aria-pressed={c.id === concept}
            title={c.title}
            onClick={() => setConcept(c.id)}
          >
            {c.label}
          </button>
        ))}
      </div>

      <div className="flex items-center justify-end gap-3">
        {loading && (
          <span className="ui-label flex items-center gap-2">
            <span className="ui-dot ui-dot-busy" /> computing
          </span>
        )}
        {error && (
          <span className="max-w-[360px] truncate text-[11px]" style={{ color: "var(--ui-bad)" }} role="alert">
            {error}
          </span>
        )}
        <BoardPill />
      </div>
    </header>
  );
}
