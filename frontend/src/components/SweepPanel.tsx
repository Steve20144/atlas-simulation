import { useState } from "react";
import { api } from "../api";
import { useTiltlabStore } from "../store";
import type { SweepRequestBody, SweepResponse } from "../types";
import { fmt } from "./format";
import SweepTable from "./SweepTable";

const btn = "rounded border border-slate-600 px-2 py-0.5 text-xs hover:bg-slate-700 disabled:opacity-40";
const input = "w-14 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-xs tabular-nums";
const TILT_MODES = ["forward", "aft", "alternating", "outer_fwd_inner_aft", "outer_aft_inner_fwd", "inward", "outward"] as const;
const GROUPINGS: { id: NonNullable<SweepRequestBody["foil_grouping"]>; label: string; pow: number }[] = [
  { id: "same", label: "one angle for both foils", pow: 1 },
  { id: "per_pair", label: "segmented foil: one angle per motor pair", pow: 4 },
  { id: "left_right", label: "left and right foil independent", pow: 2 },
];
const RANKS: { id: NonNullable<SweepRequestBody["rank_by"]>; label: string }[] = [
  { id: "control", label: "most control authority" },
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

/** Sweep the foil deflection (or raw fan tilt when the scenario has no foils); every candidate is
 * checked for level hover and for control authority on roll, pitch and yaw, then ranked. */
export default function SweepPanel() {
  const scenario = useTiltlabStore((s) => s.scenario);
  const concept = useTiltlabStore((s) => s.concept);
  const collective = useTiltlabStore((s) => s.collective);
  const applySweepCandidate = useTiltlabStore((s) => s.applySweepCandidate);
  const hasFoils = (scenario.foils?.length ?? 0) > 0;
  const [start, setStart] = useState(hasFoils ? 45 : 0);
  const [stop, setStop] = useState(hasFoils ? 150 : 45);
  const [step, setStep] = useState(hasFoils ? 15 : 5);
  const [grouping, setGrouping] = useState<NonNullable<SweepRequestBody["foil_grouping"]>>(hasFoils ? "per_pair" : "same");
  const [mode, setMode] = useState<(typeof TILT_MODES)[number]>("forward");
  const [perPair, setPerPair] = useState(false);
  const [minHeadroom, setMinHeadroom] = useState(0.1);
  const [minRollAcc, setMinRollAcc] = useState(8);
  const [minPitchAcc, setMinPitchAcc] = useState(8);
  const [minYawAcc, setMinYawAcc] = useState(2.5);
  const [maxCoupling, setMaxCoupling] = useState(0.3);
  const [pitches, setPitches] = useState("0");
  const [rankBy, setRankBy] = useState<NonNullable<SweepRequestBody["rank_by"]>>("control");
  const [result, setResult] = useState<SweepResponse | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");

  const values = range(Math.min(start, stop), Math.max(start, stop), step > 0 ? step : 15);
  const pow = hasFoils ? (GROUPINGS.find((g) => g.id === grouping)?.pow ?? 1) : perPair ? 4 : 1;
  const nPitch = Math.max(1, pitches.split(",").filter((p) => p.trim() !== "").length);
  const count = Math.pow(values.length, pow) * nPitch;

  const run = async () => {
    setBusy(true);
    setError("");
    try {
      setResult(
        await api.sweep({
          scenario, concept, collective: collective ?? undefined, tilts_deg: values,
          variable: hasFoils ? "foil" : "tilt", foil_grouping: grouping, azimuth_mode: mode, per_pair: perPair,
          min_headroom: minHeadroom, min_yaw_Nm: 0, min_roll_accel: minRollAcc, min_pitch_accel: minPitchAcc,
          min_yaw_accel: minYawAcc, max_coupling: maxCoupling, rank_by: rankBy, top: 12,
          hover_pitch_deg: pitches.split(",").map((p) => Number(p.trim())).filter((p) => Number.isFinite(p)),
        }),
      );
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setBusy(false);
    }
  };

  const d = result?.diagnostics;
  return (
    <div className="flex flex-col gap-1 rounded border border-slate-800 p-2" data-testid="sweep-panel">
      <h3 className="text-xs font-semibold">{hasFoils ? "Foil sweep" : "Tilt sweep"}</h3>
      <p className="text-[10px] text-slate-400">
        Tries every {hasFoils ? "foil deflection" : "wing-fan tilt"} in the grid. A candidate passes when it hovers level and gives the pilot at least the angular acceleration you ask for on roll, pitch and yaw (attainable torque over inertia, at hover) without leaking into the other axes. Rank by control authority to find the deflector set that is easiest to fly.
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
      </div>
      <div className="flex flex-wrap items-center gap-1 text-[11px]">
        <span>min headroom</span>
        <input className={input} type="number" value={minHeadroom} min={0} max={1} step={0.05} onChange={(e) => setMinHeadroom(Number(e.target.value))} aria-label="min headroom" />
        <span title="rad/s² at hover; 0 disables the check">min roll</span>
        <input className={input} type="number" value={minRollAcc} min={0} step={0.5} onChange={(e) => setMinRollAcc(Number(e.target.value))} aria-label="min roll acceleration" />
        <span>pitch</span>
        <input className={input} type="number" value={minPitchAcc} min={0} step={0.5} onChange={(e) => setMinPitchAcc(Number(e.target.value))} aria-label="min pitch acceleration" />
        <span>yaw rad/s²</span>
        <input className={input} type="number" value={minYawAcc} min={0} step={0.5} onChange={(e) => setMinYawAcc(Number(e.target.value))} aria-label="min yaw acceleration" />
        <span title="off-axis leakage fraction; 1 disables the check">max coupling</span>
        <input className={input} type="number" value={maxCoupling} min={0} max={1} step={0.05} onChange={(e) => setMaxCoupling(Number(e.target.value))} aria-label="max coupling" />
        <span title="hover attitudes to try, nose-up degrees, comma separated: the airframe hovers pitched and PX4's body frame is that hover frame">hover pitch°</span>
        <input className={input + " w-24"} type="text" value={pitches} onChange={(e) => setPitches(e.target.value)} aria-label="hover pitch list" />
        <span>rank by</span>
        <select className={input + " w-40"} value={rankBy} onChange={(e) => setRankBy(e.target.value as typeof rankBy)} aria-label="rank by">
          {RANKS.map((r) => <option key={r.id} value={r.id}>{r.label}</option>)}
        </select>
        <button className={btn} disabled={busy || count === 0 || count > 5000} onClick={() => void run()}>
          {busy ? "running" : `run ${count} candidates`}
        </button>
      </div>
      {error && <p className="text-[10px] text-rose-300">{error}</p>}
      {result && d && (
        <>
          <p className="text-[10px] text-slate-400">
            {result.n_feasible} of {result.n_evaluated} geometries hover level and meet every control check, {fmt(result.elapsed_ms, 0)} ms. Feasible rows first, ranked by {RANKS.find((r) => r.id === rankBy)?.label}.
          </p>
          {result.n_feasible === 0 && (
            <p className="text-[10px] text-rose-300">
              {d.n_controllable > 0
                ? `${d.n_controllable} geometries hover level and steer every axis but miss your thresholds (most often: ${d.most_common_reason_near_miss ?? topReason(result)}). Lower the minimum accelerations or headroom to see them.`
                : `No geometry in this grid can hover level and steer every axis. Most common reason: ${d.most_common_reason_near_miss ?? topReason(result)}.${hasFoils && grouping === "same" ? " With one angle for both foils only 90 deg (straight down) hovers level; use the segmented grouping." : hasFoils && grouping === "left_right" ? " Different left and right angles leave a net yaw moment; use the segmented grouping." : !hasFoils ? " Try the alternating fore-aft mode or per-pair angles." : ""}`}
            </p>
          )}
          <SweepTable
            result={result}
            angleLabel={hasFoils ? (grouping === "left_right" ? "left / right deg" : "deflection outer to inner") : "pair tilts"}
            leftRight={hasFoils && grouping === "left_right"}
            onApply={applySweepCandidate}
          />
        </>
      )}
    </div>
  );
}
