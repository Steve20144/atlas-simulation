import { useState } from "react";
import { api } from "../api";
import { useTiltlabStore } from "../store";
import type { SweepCandidate, SweepResponse } from "../types";
import { fmt } from "./format";

const btn = "rounded border border-slate-600 px-2 py-0.5 text-xs hover:bg-slate-700 disabled:opacity-40";
const input = "w-14 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-xs tabular-nums";
const MODES = ["forward", "aft", "alternating", "outer_fwd_inner_aft", "outer_aft_inner_fwd", "inward", "outward"] as const;

function range(start: number, stop: number, step: number): number[] {
  const out: number[] = [];
  if (step <= 0) return [start];
  for (let t = start; t <= stop + 1e-9 && out.length < 200; t += step) out.push(Math.round(t * 100) / 100);
  return out;
}

/** Tilt sweep: grid over wing-fan tilt, ranked by hover power among candidates that keep control. */
export default function SweepPanel() {
  const scenario = useTiltlabStore((s) => s.scenario);
  const concept = useTiltlabStore((s) => s.concept);
  const collective = useTiltlabStore((s) => s.collective);
  const applyFanAngles = useTiltlabStore((s) => s.applyFanAngles);
  const [start, setStart] = useState(0);
  const [stop, setStop] = useState(45);
  const [step, setStep] = useState(5);
  const [mode, setMode] = useState<(typeof MODES)[number]>("forward");
  const [perPair, setPerPair] = useState(false);
  const [minHeadroom, setMinHeadroom] = useState(0.2);
  const [minYaw, setMinYaw] = useState(0);
  const [result, setResult] = useState<SweepResponse | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");

  const lo = Math.min(start, stop);
  const hi = Math.max(start, stop);
  const tilts = range(lo, hi, step > 0 ? step : 5);
  const count = perPair ? Math.pow(tilts.length, 4) : tilts.length;
  const topReason = (r: SweepResponse) => {
    const counts = new Map<string, number>();
    for (const c of r.candidates) for (const reason of c.reasons) counts.set(reason, (counts.get(reason) ?? 0) + 1);
    return [...counts.entries()].sort((a, b) => b[1] - a[1])[0]?.[0] ?? "";
  };

  const run = async () => {
    setBusy(true);
    setError("");
    try {
      setResult(
        await api.sweep({
          scenario, concept, collective: collective ?? undefined, tilts_deg: tilts, azimuth_mode: mode,
          per_pair: perPair, min_headroom: minHeadroom, min_yaw_Nm: minYaw, top: 12,
        }),
      );
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="flex flex-col gap-1 rounded border border-slate-800 p-2" data-testid="sweep-panel">
      <h3 className="text-xs font-semibold">Tilt sweep</h3>
      <div className="flex flex-wrap items-center gap-1 text-[11px]">
        <span>tilt from</span>
        <input className={input} type="number" value={start} min={0} max={90} onChange={(e) => setStart(Number(e.target.value))} aria-label="sweep start" />
        <span>to</span>
        <input className={input} type="number" value={stop} min={0} max={90} onChange={(e) => setStop(Number(e.target.value))} aria-label="sweep stop" />
        <span>step</span>
        <input className={input} type="number" value={step} min={0.5} step={0.5} onChange={(e) => setStep(Number(e.target.value))} aria-label="sweep step" />
        <select className={input + " w-20"} value={mode} onChange={(e) => setMode(e.target.value as (typeof MODES)[number])} aria-label="azimuth mode">
          {MODES.map((m) => <option key={m} value={m}>{m}</option>)}
        </select>
        <label className="flex items-center gap-1"><input type="checkbox" checked={perPair} onChange={(e) => setPerPair(e.target.checked)} /> per pair</label>
        <span>min headroom</span>
        <input className={input} type="number" value={minHeadroom} min={0} max={1} step={0.05} onChange={(e) => setMinHeadroom(Number(e.target.value))} aria-label="min headroom" />
        <span>min yaw N m</span>
        <input className={input} type="number" value={minYaw} min={0} step={0.1} onChange={(e) => setMinYaw(Number(e.target.value))} aria-label="min yaw" />
        <button className={btn} disabled={busy || count === 0 || count > 5000} onClick={() => void run()}>
          {busy ? "running" : `run ${count} candidates`}
        </button>
      </div>
      {error && <p className="text-[10px] text-rose-300">{error}</p>}
      {result && (
        <>
          <p className="text-[10px] text-slate-400">
            {result.n_feasible} of {result.n_evaluated} geometries can hover and control every axis under this concept, {fmt(result.elapsed_ms, 0)} ms. Feasible rows first, ranked by hover power.
          </p>
          {result.n_feasible === 0 && (
            <p className="text-[10px] text-rose-300">
              No feasible geometry in this grid. Most common reason: {topReason(result)}. Try another azimuth mode (alternating fore-aft) or per-pair angles.
            </p>
          )}
          <table className="w-full text-[10px] tabular-nums">
            <thead>
              <tr className="text-slate-400"><th className="text-left">pair tilts</th><th>W</th><th>headroom</th><th>yaw N m</th><th>roll N m</th><th>score</th><th></th></tr>
            </thead>
            <tbody>
              {result.candidates.map((c: SweepCandidate, i) => (
                <tr key={i} className={c.feasible ? "" : "text-slate-500"} title={c.reasons.join("; ")}>
                  <td className="text-left">{c.pair_tilts_deg.join("/")}</td>
                  <td className="text-right">{fmt(c.power_W, 0)}</td>
                  <td className="text-right">{fmt(c.headroom, 2)}</td>
                  <td className="text-right">{c.yaw_Nm === null ? "-" : fmt(c.yaw_Nm, 2)}</td>
                  <td className="text-right">{c.roll_Nm === null ? "-" : fmt(c.roll_Nm, 2)}</td>
                  <td className="text-right">{fmt(c.score, 2)}</td>
                  <td>
                    <button className={btn} onClick={() => applyFanAngles(c.tilts_deg, c.azimuths_deg)}>apply</button>
                    {!c.feasible && <span className="ml-1 text-[9px] text-rose-300">{c.reasons[0]?.split(" (")[0]}</span>}
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </>
      )}
    </div>
  );
}
