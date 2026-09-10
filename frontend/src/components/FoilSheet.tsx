import { useEffect, useState } from "react";
import { api } from "../api";
import { useTiltlabStore } from "../store";
import type { FoilSheetRow } from "../types";

const btn = "rounded border border-slate-600 px-2 py-0.5 text-xs hover:bg-slate-700 disabled:opacity-40";

/**
 * Foil design sheet: for each motor, the chosen deflection translated into what to model in the
 * CAD (exhaust direction and pressure point in the CAD frame, change from the as-built 45 deg).
 */
export default function FoilSheet() {
  const scenario = useTiltlabStore((s) => s.scenario);
  const [rows, setRows] = useState<FoilSheetRow[]>([]);
  const [status, setStatus] = useState("");
  const units = scenario.frame.cad_units;
  const ppKey = `pressure_point_cad_${units}`;
  const motorKey = `motor_centre_cad_${units}`;

  useEffect(() => {
    if (!scenario.foils?.length) return;
    const t = setTimeout(() => {
      api.foilSheet(scenario).then((r) => setRows(r.rows)).catch((e: Error) => setStatus(e.message));
    }, 300);
    return () => clearTimeout(t);
  }, [scenario]);

  if (!scenario.foils?.length) return null;
  const fmtPt = (v: unknown) => (Array.isArray(v) ? v.map((x) => Number(x).toFixed(0)).join(", ") : "-");

  return (
    <details className="rounded border border-slate-800 p-2 text-[11px]" data-testid="foil-sheet">
      <summary className="cursor-pointer text-xs font-semibold">Foil design sheet (for the CAD)</summary>
      <p className="mt-1 text-slate-400">
        CAD frame: forward {scenario.frame.cad_forward_axis}, up {scenario.frame.cad_up_axis}, {units}. Model each
        foil segment so its exit sends the jet along the exhaust direction; the change column is the rotation to
        apply to the as-built 45 deg foil about the lateral axis (positive turns the exit further down and forward).
      </p>
      <table className="mt-1 w-full tabular-nums">
        <thead>
          <tr className="text-slate-400">
            <th className="text-left">motor</th>
            <th>side</th>
            <th>deflection</th>
            <th>change</th>
            <th className="text-left">exhaust dir (CAD)</th>
            <th className="text-left">pressure point ({units})</th>
            <th className="text-left">motor centre ({units})</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => (
            <tr key={r.rotor} className="border-t border-slate-800">
              <td>{r.rotor}</td>
              <td className="text-center">{r.side}</td>
              <td className="text-right">{r.deflection_deg.toFixed(1)}</td>
              <td className="text-right">{r.change_from_as_built_deg >= 0 ? "+" : ""}{r.change_from_as_built_deg.toFixed(1)}</td>
              <td>{r.exhaust_dir_cad}</td>
              <td>{fmtPt(r[ppKey])}</td>
              <td>{fmtPt(r[motorKey])}</td>
            </tr>
          ))}
        </tbody>
      </table>
      <div className="mt-1 flex items-center gap-2">
        <button
          className={btn}
          onClick={() =>
            api.exportFoilSheet(scenario).then((r) => setStatus(`wrote ${r.md_path} and ${r.csv_path}`)).catch((e: Error) => setStatus(e.message))
          }
        >
          export sheet (.md + .csv)
        </button>
        {status && <span className="break-all text-[10px] text-slate-400">{status}</span>}
      </div>
    </details>
  );
}
