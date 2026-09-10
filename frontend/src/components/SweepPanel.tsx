import { useState } from "react";
import { api } from "../api";
import { useTiltlabStore } from "../store";
import type { SweepCandidate, SweepRequestBody, SweepResponse } from "../types";
import { fmt } from "./format";

const btn = "rounded border border-slate-600 px-2 py-0.5 text-xs hover:bg-slate-700 disabled:opacity-40";
const input = "w-14 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-xs tabular-nums";
const TILT_MODES = ["forward", "aft", "alternating", "outer_fwd_inner_aft", "outer_aft_inner_fwd", "inward", "outward"] as const;
const GROUPINGS: { id: NonNullable<SweepRequestBody["foil_grouping"]>; label: string; pow: number }[] = [
  { id: "same", label: "one angle for both foils", pow: 1 },
  { id: "per_pair", label: "segmented foil: one angle per motor pair", pow: 4 },
  { id: "left_right", label: "left and right foil independent", pow: 2 },
];

const RANKS: { id: NonNullable<SweepRequestBody["rank_by"]>; label: string }[] = [
  { id: "power", label: "lowest hover power" },
  { id: "yaw", label: "most yaw authority" },
  { id: "yaw_per_kW", label: "most yaw per kW" },
  { id: "headroom", label: "most headroom" },
  { id: "score", label: "composite score" },
];

function range(start: number, stop: number, step: number): number[] {
  const out: number[] = [];
  for (let t = start; t <= stop + 1e-9 && out.length < 200; t += step) out.push(Math.round(t * 100) / 100);
  return out;
}

function topReason(r: SweepResponse): string {
  const counts = new Map<string, number>();
  for (const c of r.candidates) for (const reason of c.reasons) counts.set(reason, (counts.get(reason) ?? 0) + 1);
  return [...counts.entries()].sort((a, b) => b[1] - a[1])[0]?.[0] ?? "";
}

/** Sweep the foil deflection (or raw fan tilt when the scenario has no foils), ranked by hover power. */
export default function SweepPanel() {
  const scenario = useTiltlabStore((s) => s.scenario);
  const concept = useTiltlabStore((s) => s.concept);
  const collective = useTiltlabStore((s) => s.collective);
  const applySweepCandidate = useTiltlabStore((s) => s.applySweepCandidate);
  const hasFoils = (scenario.foils?.length ?? 0) > 0;
  const [start, setStart] = useState(0);
  const [stop, setStop] = useState(hasFoils ? 180 : 45);
  const [step, setStep] = useState(hasFoils ? 15 : 5);
  const [grouping, setGrouping] = useState<NonNullable<SweepRequestBody["foil_grouping"]>>("same");
  const [mode, setMode] = useState<(typeof TILT_MODES)[number]>("forward");
  const [perPair, setPerPair] = useState(false);
  const [minHeadroom, setMinHeadroom] = useState(0.1);
  const [minYaw, setMinYaw] = useState(0);
  const [rankBy, setRankBy] = useState<NonNullable<SweepRequestBody["rank_by"]>>("power");
  const [result, setResult] = useState<SweepResponse | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");

  const values = range(Math.min(start, stop), Math.max(start, stop), step > 0 ? step : 15);
  const pow = hasFoils ? (GROUPINGS.find((g) => g.id === grouping)?.pow ?? 1) : perPair ? 4 : 1;
  const count = Math.pow(values.length, pow);

  const run = async () => {
    setBusy(true);
    setError("");
    try {
      setResult(
        await api.sweep({
          scenario, concept, collective: collective ?? undefined, tilts_deg: values,
          variable: hasFoils ? "foil" : "tilt", foil_grouping: grouping, azimuth_mode: mode, per_pair: perPair,
          min_headroom: minHeadroom, min_yaw_Nm: minYaw, rank_by: rankBy, top: 12,
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
      <h3 className="text-xs font-semibold">{hasFoils ? "Foil sweep" : "Tilt sweep"}</h3>
      <p className="text-[10px] text-slate-400">
        {hasFoils
          ? "Tries every foil deflection in the grid, keeps the ones that can hover level and steer all axes, and ranks them by hover power."
          : "Tries every wing-fan tilt in the grid, keeps the ones that can hover level and steer all axes, and ranks them by hover power."}
      </p>
      <div className="flex flex-wrap items-center gap-1 text-[11px]">
        <span>{hasFoils ? "deflection from" : "tilt from"}</span>
        <input className={input} type="number" value={start} min={0} max={180} onChange={(e) => setStart(Number(e.target.value))} aria-label="sweep start" />
        <span>to</span>
        <input className={input} type="number" value={stop} min={0} max={180} onChange={(e) => setStop(Number(e.target.value))} aria-label="sweep stop" />
        <span>step</span>
        <input className={input} type="number" value={step} min={0.5} step={0.5} onChange={(e) => setStep(Number(e.target.value))} aria-label="sweep step" />
        {hasFoils ? (
          <select className={input + " w-56"} value={grouping} onChange={(e) => setGrouping(e.target.value as typeof grouping)} aria-label="foil grouping">
            {GROUPINGS.map((g) => <option key={g.id} value={g.id}>{g.label}</option>)}
          </select>
        ) : (
          <>
            <select className={input + " w-32"} value={mode} onChange={(e) => setMode(e.target.value as (typeof TILT_MODES)[number])} aria-label="azimuth mode">
              {TILT_MODES.map((m) => <option key={m} value={m}>{m}</option>)}
            </select>
            <label className="flex items-center gap-1"><input type="checkbox" checked={perPair} onChange={(e) => setPerPair(e.target.checked)} /> per pair</label>
          </>
        )}
        <span>min headroom</span>
        <input className={input} type="number" value={minHeadroom} min={0} max={1} step={0.05} onChange={(e) => setMinHeadroom(Number(e.target.value))} aria-label="min headroom" />
        <span>min yaw N m</span>
        <input className={input} type="number" value={minYaw} min={0} step={0.1} onChange={(e) => setMinYaw(Number(e.target.value))} aria-label="min yaw" />
        <span>rank by</span>
        <select className={input + " w-36"} value={rankBy} onChange={(e) => setRankBy(e.target.value as typeof rankBy)} aria-label="rank by">
          {RANKS.map((r) => <option key={r.id} value={r.id}>{r.label}</option>)}
        </select>
        <button className={btn} disabled={busy || count === 0 || count > 5000} onClick={() => void run()}>
          {busy ? "running" : `run ${count} candidates`}
        </button>
      </div>
      {error && <p className="text-[10px] text-rose-300">{error}</p>}
      {result && (
        <>
          <p className="text-[10px] text-slate-400">
            {result.n_feasible} of {result.n_evaluated} geometries can hover and control every axis under this concept, {fmt(result.elapsed_ms, 0)} ms. Feasible rows first, ranked by {RANKS.find((r) => r.id === rankBy)?.label}.
          </p>
          {result.n_feasible === 0 && (
            <p className="text-[10px] text-rose-300">
              {result.diagnostics.n_controllable > 0
                ? `${result.diagnostics.n_controllable} of these geometries can hover level and steer every axis but miss your thresholds: the best of them reaches headroom ${fmt(result.diagnostics.best_headroom_controllable ?? 0, 2)} and yaw ${fmt(result.diagnostics.best_yaw_controllable ?? 0, 1)} N m. Lower min headroom or min yaw to see them${result.diagnostics.best_headroom_controllable !== null && result.diagnostics.best_headroom_controllable < 0.15 ? " (headroom is capped by the estimated mass until the real weights are in)" : ""}.`
                : `No geometry in this grid can hover level and steer every axis. Most common reason: ${result.diagnostics.most_common_reason_near_miss ?? topReason(result)}.${hasFoils && grouping === "same" ? " With one angle for both foils only 90 deg (straight down) hovers level; use the segmented grouping for yaw." : hasFoils && grouping === "left_right" ? " Different left and right angles leave a net yaw moment; use the segmented grouping." : !hasFoils ? " Try the alternating fore-aft mode or per-pair angles." : ""}`}
            </p>
          )}
          <table className="w-full text-[10px] tabular-nums">
            <thead>
              <tr className="text-slate-400">
                <th className="text-left">{hasFoils ? (grouping === "left_right" ? "left / right deg" : "deflection outer to inner") : "pair tilts"}</th>
                <th>W</th><th>headroom</th><th>yaw N m</th><th>roll N m</th><th>score</th><th></th>
              </tr>
            </thead>
            <tbody>
              {result.candidates.map((c: SweepCandidate, i) => (
                <tr key={i} className={c.feasible ? "" : "text-slate-500"} title={c.reasons.join("; ")}>
                  <td className="text-left">
                    {hasFoils && grouping === "left_right" ? `${fmt(c.left_deg ?? 0, 0)} / ${fmt(c.right_deg ?? 0, 0)}` : c.pair_tilts_deg.join("/")}
                  </td>
                  <td className="text-right">{fmt(c.power_W, 0)}</td>
                  <td className="text-right">{fmt(c.headroom, 2)}</td>
                  <td className="text-right">{c.yaw_Nm === null ? "-" : fmt(c.yaw_Nm, 2)}</td>
                  <td className="text-right">{c.roll_Nm === null ? "-" : fmt(c.roll_Nm, 2)}</td>
                  <td className="text-right">{fmt(c.score, 2)}</td>
                  <td>
                    <button className={btn} onClick={() => applySweepCandidate(c)}>apply</button>
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
