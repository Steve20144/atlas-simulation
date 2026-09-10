import type { CouplingMetrics } from "../types";
import { fmt } from "./format";

/** Leakage fraction matrix (backend coupling.leakage_fraction) with its axis labels. */
export function normaliseCoupling(c: CouplingMetrics): { matrix: number[][]; labels: string[] } {
  const matrix = c.leakage_fraction ?? [];
  const labels = c.axes && c.axes.length === matrix.length ? c.axes : matrix.map((_, i) => String(i));
  return { matrix, labels };
}

function cellClass(v: number, diagonal: boolean): string {
  if (diagonal) return "text-slate-200";
  const a = Math.abs(v);
  if (a >= 0.5) return "text-rose-300";
  if (a >= 0.2) return "text-amber-300";
  return "text-slate-500";
}

/** Axis coupling matrix (row = commanded axis, column = delivered); off-diagonal coloured by magnitude. */
export default function CouplingTable({ coupling }: { coupling: CouplingMetrics }) {
  const { matrix, labels } = normaliseCoupling(coupling);
  if (matrix.length === 0) return <p className="text-xs text-slate-500">no coupling data</p>;
  return (
    <div className="overflow-x-auto">
      <table className="text-[11px]">
        <thead>
          <tr className="text-slate-400">
            <th />
            {labels.map((l) => (
              <th key={l} className="px-1 text-right font-medium">
                {l}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {matrix.map((row, i) => (
            <tr key={labels[i] ?? i}>
              <td className="pr-1 text-slate-400">{labels[i] ?? i}</td>
              {row.map((v, j) => (
                <td key={j} className={`px-1 text-right tabular-nums ${cellClass(v, i === j)}`}>
                  {fmt(v)}
                </td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
      {coupling.max_offaxis_fraction !== undefined && (
        <p className="mt-1 text-[10px] text-slate-500">
          max off-axis fraction {fmt(coupling.max_offaxis_fraction, 3)}
        </p>
      )}
    </div>
  );
}
