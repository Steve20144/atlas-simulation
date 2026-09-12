import type { SweepResponse } from "../types";
import { fmt } from "./format";

function topReason(r: SweepResponse): string {
  const counts = new Map<string, number>();
  for (const c of r.candidates) for (const reason of c.reasons) counts.set(reason, (counts.get(reason) ?? 0) + 1);
  return [...counts.entries()].sort((a, b) => b[1] - a[1])[0]?.[0] ?? "";
}

interface Props {
  result: SweepResponse;
  rankLabel: string;
  hasFoils: boolean;
  grouping: string;
}

/** One-line outcome of a sweep plus, when nothing passed, why and what to change. */
export default function SweepSummary({ result, rankLabel, hasFoils, grouping }: Props) {
  const d = result.diagnostics;
  const hint =
    hasFoils && grouping === "same"
      ? " With one angle for both foils only 90 deg (straight down) hovers level; use the segmented grouping."
      : hasFoils && grouping === "left_right"
        ? " Different left and right angles leave a net yaw moment; use the segmented grouping."
        : !hasFoils
          ? " Try the alternating fore-aft mode or per-pair angles."
          : "";
  return (
    <>
      <p className="text-[10px] text-slate-400">
        {result.n_feasible} of {result.n_evaluated} geometries hover level and meet every control check, {fmt(result.elapsed_ms, 0)} ms. Feasible rows first, ranked by {rankLabel}.
        {result.truncated ? " Grid truncated at the candidate cap; narrow it." : ""}
      </p>
      {result.n_feasible === 0 && (
        <p className="text-[10px] text-rose-300">
          {d.n_controllable > 0
            ? `${d.n_controllable} geometries hover level and steer every axis but miss your thresholds (most often: ${d.most_common_reason_near_miss ?? topReason(result)}). Lower the minimum accelerations or headroom to see them.`
            : `No geometry in this grid can hover level and steer every axis. Most common reason: ${d.most_common_reason_near_miss ?? topReason(result)}.${hint}`}
        </p>
      )}
    </>
  );
}
