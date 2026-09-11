import type { ControlMetrics } from "../types";
import { fmt } from "./format";

/** What the pilot gets per axis at hover: angular acceleration from the attainable torque over the
 * inertia, the share of that torque available before PX4 has to desaturate, time to 10 deg of
 * bank, and the fore-aft or lateral force that leaks out of a torque command. */
export default function ControlTable({ control }: { control: ControlMetrics }) {
  const axes = Object.entries(control.axes);
  return (
    <div className="flex flex-col gap-1 text-[10px]">
      <table className="w-full tabular-nums">
        <thead>
          <tr className="text-slate-400">
            <th className="text-left">axis</th>
            <th title="attainable torque in both directions">N m</th>
            <th title="torque over the inertia about this axis">rad/s²</th>
            <th title="required by the current thresholds">need</th>
            <th title="time to 10 deg from rest under the full attainable torque">to 10°</th>
            <th title="share of the torque reachable before a fan hits 0 or 100 percent">linear</th>
            <th title="fore-aft or lateral force leaking when 20 percent of the axis is commanded, fraction of weight">surge</th>
          </tr>
        </thead>
        <tbody>
          {axes.map(([name, a]) => {
            const ok = a.attainable && a.accel_rad_s2 >= a.required_accel_rad_s2;
            return (
              <tr key={name} className={ok ? "" : "text-rose-300"}>
                <td className="text-left">{name}{control.weakest_axis === name ? " (weakest)" : ""}</td>
                <td className="text-right">{a.attainable ? fmt(a.torque_Nm, 1) : "-"}</td>
                <td className="text-right">{a.attainable ? fmt(a.accel_rad_s2, 1) : "-"}</td>
                <td className="text-right">{fmt(a.required_accel_rad_s2, 1)}</td>
                <td className="text-right">{a.time_to_10deg_s === null ? "-" : `${fmt(a.time_to_10deg_s, 2)} s`}</td>
                <td className="text-right">{a.attainable ? fmt(a.linear_fraction, 2) : "-"}</td>
                <td className="text-right">{fmt(a.surge_leak_frac_of_weight, 2)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <p className="text-slate-400">
        Off-axis coupling {fmt(control.checks.find((c) => c.name === "off-axis coupling")?.value ?? 0, 2)} (max {fmt(control.requirements.max_coupling, 2)}).
        Fan spool {fmt(control.fan_lag_s * 1000, 0)} ms limits the rate loop to about {fmt(control.rate_bandwidth_rad_s, 1)} rad/s.
        Control score {fmt(control.score, 2)}: {control.pass ? "every check passes" : "a check fails"}.
        {control.inertia_placeholder ? " Inertia is the box placeholder until the CAD mass properties are in; compare geometries relatively." : ""}
      </p>
    </div>
  );
}
