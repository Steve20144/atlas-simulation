import type { AuthorityKey, AuthorityMetrics } from "../types";
import { fmt } from "./format";

const KEYS: AuthorityKey[] = ["roll", "pitch", "yaw", "Fx", "Fy", "Fz"];

/** Backend badge vocabulary (core/metrics.py BADGE_STOCK / BADGE_FA) plus UI heuristics. */
const BADGE_CLASS: Record<string, string> = {
  stock_px4: "bg-emerald-800 text-emerald-100",
  good: "bg-emerald-800 text-emerald-100",
  needs_fully_actuated_controller: "bg-amber-800 text-amber-100",
  weak: "bg-amber-800 text-amber-100",
  none: "bg-rose-900 text-rose-100",
  unattainable: "bg-rose-900 text-rose-100",
};

const BADGE_SHORT: Record<string, string> = {
  stock_px4: "stock",
  needs_fully_actuated_controller: "fully act.",
};

/** Capability badge coloured by the backend's verdict string. */
export function Badge({ label }: { label: string }) {
  const key = label.toLowerCase();
  const cls = BADGE_CLASS[key] ?? "bg-slate-700 text-slate-200";
  return (
    <span
      className={`rounded px-1.5 py-0.5 text-[10px] font-medium uppercase ${cls}`}
      data-testid="badge"
      title={label}
    >
      {BADGE_SHORT[key] ?? label}
    </span>
  );
}

/** Per-axis control authority (plus / minus, per watt) with a capability badge. */
export default function AuthorityTable({ authority }: { authority: AuthorityMetrics }) {
  return (
    <table className="w-full text-xs">
      <thead>
        <tr className="text-slate-400">
          <th className="text-left font-medium">axis</th>
          <th className="text-right font-medium">+</th>
          <th className="text-right font-medium">-</th>
          <th className="text-left font-medium">unit</th>
          <th className="text-right font-medium">+ per W</th>
          <th className="text-left font-medium">badge</th>
        </tr>
      </thead>
      <tbody>
        {KEYS.filter((k) => authority[k]).map((k) => {
          const a = authority[k]!;
          const unattainable = a.plus_attainable === false || a.minus_attainable === false;
          return (
            <tr key={k} className="border-t border-slate-800">
              <td className="py-0.5">{k}</td>
              <td className="text-right tabular-nums">{fmt(a.plus)}</td>
              <td className="text-right tabular-nums">{fmt(a.minus)}</td>
              <td className="text-slate-400">{a.unit}</td>
              <td className="text-right tabular-nums">{fmt(a.plus_per_W, 4)}</td>
              <td className="flex gap-1">
                <Badge label={String(a.badge ?? "")} />
                {unattainable && <Badge label="unattainable" />}
              </td>
            </tr>
          );
        })}
      </tbody>
    </table>
  );
}
