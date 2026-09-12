import type { SweepRequestBody } from "../types";

const input = "w-14 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-xs tabular-nums";

export type NosePairing = NonNullable<SweepRequestBody["nose_pairing"]>;
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
}

export const DEFAULT_NOSE_GRID: NoseGrid = { enabled: false, start: -30, stop: 30, step: 10, pairing: "opposed" };

/** Signed sideways tilt grid for the nose fans (degrees, negative left, positive right). */
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

/** Row of the sweep panel that adds the two nose (centreline) fans to the sweep. They tilt
 * sideways about the aircraft's longitudinal axis: negative degrees send the jet to the left,
 * positive to the right, 0 is straight down. */
export default function SweepNoseControls({ grid, onChange }: Props) {
  const set = (patch: Partial<NoseGrid>) => onChange({ ...grid, ...patch });
  return (
    <div className="flex flex-wrap items-center gap-1 text-[11px]" data-testid="sweep-nose">
      <label className="flex items-center gap-1" title="tilt the two nose fans sideways (about the aircraft's longitudinal axis) as part of the sweep">
        <input type="checkbox" checked={grid.enabled} onChange={(e) => set({ enabled: e.target.checked })} aria-label="sweep nose fans sideways" />
        nose fans sideways
      </label>
      <span title="signed degrees: negative = jet to the left, positive = to the right, 0 = straight down">from</span>
      <input className={input} type="number" value={grid.start} min={-90} max={90} disabled={!grid.enabled} onChange={(e) => set({ start: Number(e.target.value) })} aria-label="nose sweep start" />
      <span>to</span>
      <input className={input} type="number" value={grid.stop} min={-90} max={90} disabled={!grid.enabled} onChange={(e) => set({ stop: Number(e.target.value) })} aria-label="nose sweep stop" />
      <span>step</span>
      <input className={input} type="number" value={grid.step} min={0.5} step={0.5} disabled={!grid.enabled} onChange={(e) => set({ step: Number(e.target.value) })} aria-label="nose sweep step" />
      <select className={input + " w-64"} value={grid.pairing} disabled={!grid.enabled} onChange={(e) => set({ pairing: e.target.value as NosePairing })} aria-label="nose pairing">
        {NOSE_PAIRINGS.map((p) => <option key={p.id} value={p.id}>{p.label}</option>)}
      </select>
      <span className="text-slate-500">left is negative, right positive</span>
    </div>
  );
}
