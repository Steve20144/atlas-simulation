import { useState } from "react";
import { api } from "../api";
import { useTiltlabStore } from "../store";

const btn = "rounded border border-slate-600 px-2 py-0.5 text-xs hover:bg-slate-700 disabled:opacity-40";

/** Two export buttons: PX4 .params and a one-row metrics CSV, both written to exports/ by the backend. */
export default function ExportPanel() {
  const scenario = useTiltlabStore((s) => s.scenario);
  const concept = useTiltlabStore((s) => s.concept);
  const m = useTiltlabStore((s) => s.metrics);
  const [status, setStatus] = useState("");
  const run = (job: Promise<{ path: string }>) =>
    job.then((r) => setStatus(`wrote ${r.path}`)).catch((e: Error) => setStatus(e.message));
  const row = m && {
    scenario: m.scenario_name, concept: m.concept, collective: m.collective, estimated: m.estimated,
    hover: { power_W: m.hover.power_W, headroom: m.hover.headroom }, authority: m.authority,
    coupling: { max_offaxis_fraction: m.coupling.max_offaxis_fraction }, conditioning: m.conditioning, score: m.score,
  };
  return (
    <div className="flex flex-col gap-1">
      <h3 className="text-xs font-semibold">Export</h3>
      <div className="flex gap-2">
        <button className={btn} onClick={() => void run(api.exportParams(scenario, concept))}>.params</button>
        <button className={btn} disabled={!row} onClick={() => row && void run(api.exportCsv([row], scenario.meta.name))}>CSV</button>
        <button
          className={btn}
          title="gz sim model, world, PX4 airframe and README under exports/gazebo/<name>/"
          onClick={() => void api.exportGazebo(scenario).then((r) => setStatus(`wrote Gazebo harness to ${r.root}`)).catch((e: Error) => setStatus(e.message))}
        >
          Gazebo
        </button>
      </div>
      {status && <p className="break-all text-[10px] text-slate-400">{status}</p>}
    </div>
  );
}
