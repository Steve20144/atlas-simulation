import type { SweepRequestBody } from "../types";

const input = "w-14 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-xs tabular-nums";

export type NosePairing = NonNullable<SweepRequestBody["nose_pairing"]>;
export type NoseAxis = NonNullable<SweepRequestBody["nose_axis"]>;
export const NOSE_AXES: { id: NoseAxis; label: string; hint: string }[] = [
  { id: "fwd", label: "forward / aft", hint: "positive leans the thrust forward, negative aft" },
  { id: "side", label: "left / right", hint: "positive leans the thrust to the right, negative to the left" },
];
export const NOSE_PAIRINGS: { id: NosePairing; label: string; pow: number }[] = [
  { id: "opposed", label: "opposed: front one way, rear the other", pow: 1 },
  { id: "same", label: "same: both to the same side", pow: 1 },
  { id: "independent", label: "independent: every front/rear combination", pow: 2 },
];

export interface NoseGrid {
  enabled: boolean;
  start: number;
  stop: number;
  step: number;
  pairing: NosePairing;
  axis: NoseAxis;
}

export const DEFAULT_NOSE_GRID: NoseGrid = { enabled: false, start: -30, stop: 30, step: 10, pairing: "opposed", axis: "fwd" };

/** Signed tilt grid for the nose fans, degrees about the chosen axis (see NOSE_AXES). */
export function noseValues(g: NoseGrid): number[] {
  if (!g.enabled) return [];
  const lo = Math.min(g.start, g.stop);
  const hi = Math.max(g.start, g.stop);
  const step = g.step > 0 ? g.step : 10;
  const out: number[] = [];
  for (let t = lo; t <= hi + 1e-9 && out.length < 200; t += step) out.push(Math.round(t * 100) / 100);
  return out;
}

/** Number of nose settings the grid adds as a factor to the candidate count (1 when off). */
export function noseCount(g: NoseGrid): number {
  const n = noseValues(g).length;
  if (n === 0) return 1;
  return Math.pow(n, NOSE_PAIRINGS.find((p) => p.id === g.pairing)?.pow ?? 1);
}

interface Props {
  grid: NoseGrid;
  onChange: (grid: NoseGrid) => void;
}

/** Row of the sweep panel that adds the two nose (centreline) fans to the sweep. They lean
 * about one body axis: forward / aft (positive forward) or left / right (positive right); 0 is
 * straight up. Opposed pairing gives the front fan +v and the rear fan -v. */
export default function SweepNoseControls({ grid, onChange }: Props) {
  const set = (patch: Partial<NoseGrid>) => onChange({ ...grid, ...patch });
  const axis = NOSE_AXES.find((a) => a.id === grid.axis) ?? NOSE_AXES[0];
  return (
    <div className="flex flex-wrap items-center gap-1 text-[11px]" data-testid="sweep-nose">
      <label className="flex items-center gap-1" title="lean the two nose fans as part of the sweep">
        <input type="checkbox" checked={grid.enabled} onChange={(e) => set({ enabled: e.target.checked })} aria-label="sweep nose fans" />
        nose fans
      </label>
      <select className={input + " w-28"} value={grid.axis} disabled={!grid.enabled} onChange={(e) => set({ axis: e.target.value as NoseAxis })} aria-label="nose axis" title={axis.hint}>
        {NOSE_AXES.map((a) => <option key={a.id} value={a.id}>{a.label}</option>)}
      </select>
      <span title={`signed degrees: ${axis.hint}, 0 = straight up`}>from</span>
      <input className={input} type="number" value={grid.start} min={-180} max={180} disabled={!grid.enabled} onChange={(e) => set({ start: Number(e.target.value) })} aria-label="nose sweep start" />
      <span>to</span>
      <input className={input} type="number" value={grid.stop} min={-180} max={180} disabled={!grid.enabled} onChange={(e) => set({ stop: Number(e.target.value) })} aria-label="nose sweep stop" />
      <span>step</span>
      <input className={input} type="number" value={grid.step} min={0.5} step={0.5} disabled={!grid.enabled} onChange={(e) => set({ step: Number(e.target.value) })} aria-label="nose sweep step" />
      <select className={input + " w-64"} value={grid.pairing} disabled={!grid.enabled} onChange={(e) => set({ pairing: e.target.value as NosePairing })} aria-label="nose pairing">
        {NOSE_PAIRINGS.map((p) => <option key={p.id} value={p.id}>{p.label}</option>)}
      </select>
      <span className="text-slate-500">{axis.hint}</span>
    </div>
  );
}
