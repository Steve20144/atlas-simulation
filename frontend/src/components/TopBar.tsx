import { useEffect } from "react";
import { PRESETS } from "../presets";
import { useTiltlabStore } from "../store";
import type { ControlConcept } from "../types";

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

const btn = "rounded border border-slate-600 px-2 py-1 text-xs hover:bg-slate-700";
const btnActive = "rounded border border-sky-400 bg-sky-900 px-2 py-1 text-xs";

export default function TopBar() {
  const scenarioName = useTiltlabStore((s) => s.scenario.meta.name);
  const scenarioNames = useTiltlabStore((s) => s.scenarioNames);
  const concept = useTiltlabStore((s) => s.concept);
  const collective = useTiltlabStore((s) => s.collective);
  const collectiveHover = useTiltlabStore((s) => s.metrics?.collective_hover);
  const loading = useTiltlabStore((s) => s.loading);
  const error = useTiltlabStore((s) => s.error);
  const { loadScenarioNames, loadScenario, saveScenario, setConcept, setCollective, applyPreset } =
    useTiltlabStore.getState();

  useEffect(() => {
    void loadScenarioNames();
  }, [loadScenarioNames]);

  // In hover mode the slider sits where the backend's hover solution is (fraction of Fz_max).
  const sliderValue = collective ?? collectiveHover ?? 0.5;

  return (
    <header className="flex flex-wrap items-center gap-4 border-b border-slate-700 bg-slate-900 px-4 py-2">
      <h1 className="text-lg font-semibold tracking-tight">tiltlab</h1>

      <label className="flex items-center gap-2 text-xs">
        Scenario
        <select
          aria-label="Scenario"
          className="rounded border border-slate-600 bg-slate-800 px-2 py-1"
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
        <button className={btn} onClick={() => void saveScenario()} title="POST /api/scenarios/{name}">
          Save
        </button>
      </label>

      <div className="flex items-center gap-1 text-xs" role="group" aria-label="Concept">
        {CONCEPTS.map((c) => (
          <button
            key={c.id}
            className={c.id === concept ? btnActive : btn}
            aria-pressed={c.id === concept}
            title={c.title}
            onClick={() => setConcept(c.id)}
          >
            {c.label}
          </button>
        ))}
      </div>

      <label className="flex items-center gap-2 text-xs">
        Collective
        <input
          aria-label="Collective"
          type="range"
          min={0}
          max={1}
          step={0.01}
          value={sliderValue}
          onChange={(e) => setCollective(Number(e.target.value))}
        />
        <span className="w-10 tabular-nums">{sliderValue.toFixed(2)}</span>
        <button
          className={collective === null ? btnActive : btn}
          onClick={() => setCollective(null)}
          title="Let the backend solve for hover"
        >
          hover
        </button>
      </label>

      <div className="flex items-center gap-1 text-xs" role="group" aria-label="Presets">
        <span className="text-slate-400">Presets</span>
        {PRESETS.map((p) => (
          <button key={p.id} className={btn} title={p.title} onClick={() => void applyPreset(p.id)}>
            {p.label}
          </button>
        ))}
      </div>

      <div className="ml-auto text-xs">
        {loading && <span className="text-slate-400">computing...</span>}
        {error && (
          <span className="text-rose-400" role="alert">
            {error}
          </span>
        )}
      </div>
    </header>
  );
}
