import type { SweepCandidate, SweepResponse } from "../types";
import { fmt } from "./format";

const btn = "rounded border border-slate-600 px-2 py-0.5 text-xs hover:bg-slate-700 disabled:opacity-40";

function num(v: number | null | undefined, digits: number): string {
  return v === null || v === undefined ? "-" : fmt(v, digits);
}

interface Props {
  result: SweepResponse;
  angleLabel: string;
  leftRight: boolean;
  onApply: (c: SweepCandidate) => void;
}

/** Ranked sweep rows: geometry, hover power and margin, then what the pilot gets on each axis
 * (angular acceleration the attainable torque gives at hover), coupling and the control score. */
function signed(v: number): string {
  return (v > 0 ? "+" : v < 0 ? "-" : "") + fmt(Math.abs(v), 0);
}

export default function SweepTable({ result, angleLabel, leftRight, onApply }: Props) {
  const nose = result.candidates.some((c) => c.nose_tilts_deg);
  return (
    <div className="overflow-x-auto">
      <table className="w-full text-[10px] tabular-nums">
        <thead>
          <tr className="text-slate-400">
            <th className="text-left">{angleLabel}</th>
            {nose && <th title="nose fans sideways tilt, front / rear, degrees: negative left, positive right">nose F/R</th>}
            <th>W</th>
            <th>headroom</th>
            <th title="angular acceleration from the attainable roll torque over inertia">roll rad/s²</th>
            <th title="angular acceleration from the attainable pitch torque over inertia">pitch rad/s²</th>
            <th title="angular acceleration from the attainable yaw torque over inertia">yaw rad/s²</th>
            <th title="off-axis leakage when 20 percent of an axis is commanded">coupling</th>
            <th title="fore-aft or lateral force leaking out of a torque command, fraction of weight">surge</th>
            <th title="weakest axis over its requirement, penalised by coupling and surge">control</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          {result.candidates.map((c, i) => (
            <tr key={i} className={c.feasible ? "" : "text-slate-500"} title={c.reasons.join("; ")}>
              <td className="text-left">
                {leftRight ? `${fmt(c.left_deg ?? 0, 0)} / ${fmt(c.right_deg ?? 0, 0)}` : c.pair_tilts_deg.join("/")}
                {c.hover_pitch_deg ? ` @${fmt(c.hover_pitch_deg, 0)}°` : ""}
              </td>
              {nose && <td className="text-right">{c.nose_tilts_deg ? `${signed(c.nose_tilts_deg[0])} / ${signed(c.nose_tilts_deg[1])}` : "-"}</td>}
              <td className="text-right">{fmt(c.power_W, 0)}</td>
              <td className="text-right">{fmt(c.headroom, 2)}</td>
              <td className={"text-right" + (c.weakest_axis === "roll" ? " text-amber-300" : "")}>{num(c.roll_acc, 1)}</td>
              <td className={"text-right" + (c.weakest_axis === "pitch" ? " text-amber-300" : "")}>{num(c.pitch_acc, 1)}</td>
              <td className={"text-right" + (c.weakest_axis === "yaw" ? " text-amber-300" : "")}>{num(c.yaw_acc, 1)}</td>
              <td className="text-right">{num(c.coupling_max, 2)}</td>
              <td className="text-right">{num(c.surge_leak, 2)}</td>
              <td className="text-right">{num(c.control_score, 2)}</td>
              <td>
                <button className={btn} onClick={() => onApply(c)}>apply</button>
                {!c.feasible && <span className="ml-1 text-[9px] text-rose-300">{c.reasons[0]?.split(" (")[0]}</span>}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      <p className="text-[10px] text-slate-500">
        Amber marks the weakest axis of each row. Accelerations use the scenario inertia (box placeholder until the CAD mass properties are in), so compare rows relatively.
      </p>
    </div>
  );
}
