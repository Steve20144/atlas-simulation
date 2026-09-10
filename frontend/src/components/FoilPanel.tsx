import { clampTilt, describeAxis, effectiveFan, wrapAzimuth } from "../geometry";
import { useTiltlabStore } from "../store";
import FoilCard from "./FoilCard";
import FoilSheet from "./FoilSheet";

const num = "w-16 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-right tabular-nums";

/** Left panel for foil scenarios: foil deflections, centreline fans, motor details. */
export default function FoilPanel() {
  const scenario = useTiltlabStore((s) => s.scenario);
  const foilLinked = useTiltlabStore((s) => s.foilLinked);
  const hoverU = useTiltlabStore((s) => s.metrics?.hover?.u);
  const { setFoilLinked, setFoilLoss, updateFan } = useTiltlabStore.getState();
  const foils = scenario.foils ?? [];
  const foilFanIds = new Set(foils.flatMap((f) => f.fan_ids));
  const plainFans = scenario.fans.filter((f) => !foilFanIds.has(f.id));
  const loss = foils[0]?.loss_at_90deg ?? 0;

  return (
    <section className="flex h-full flex-col gap-2 overflow-auto border-r border-slate-700 bg-slate-900/60 p-2">
      <div className="flex items-baseline justify-between">
        <h2 className="text-sm font-semibold">Geometry</h2>
        <span className="text-xs text-slate-400">mass {scenario.mass.total_kg.toFixed(2)} kg</span>
      </div>
      <p className="text-[11px] leading-snug text-slate-400">
        The wing motors are horizontal and blow aft into a foil. The foil turns the jet down, so the
        force on the aircraft acts at the foil and points where the deflection sends it: 0 deg pushes
        straight forward, 90 deg is pure lift, 180 deg pushes straight back.
      </p>
      <label className="flex items-center gap-2 text-xs">
        <input type="checkbox" checked={foilLinked} onChange={(e) => setFoilLinked(e.target.checked)} />
        same angle on both foils
      </label>
      {foils.map((f) => (
        <FoilCard key={f.id} foil={f} title={`${f.id[0].toUpperCase()}${f.id.slice(1)} foil`} />
      ))}
      <label className="flex items-center gap-2 text-xs" title="fraction of motor thrust lost when the jet is turned 90 deg; scales with sin^2 of the deflection">
        <span>turning loss at 90 deg</span>
        <input
          aria-label="turning loss"
          type="number"
          min={0}
          max={1}
          step={0.01}
          className={num}
          value={loss}
          onChange={(e) => setFoilLoss(Math.min(1, Math.max(0, Number(e.target.value))))}
        />
        <span className="text-slate-500">(not measured)</span>
      </label>

      {plainFans.length > 0 && (
        <div className="rounded border border-slate-800 p-2">
          <h3 className="text-xs font-semibold">Centreline fans</h3>
          <table className="mt-1 w-full text-[11px]">
            <thead>
              <tr className="text-slate-400">
                <th className="text-left">fan</th>
                <th>tilt deg</th>
                <th>azimuth deg</th>
                <th className="text-left">thrust</th>
                <th title="hover command 0..1">u</th>
              </tr>
            </thead>
            <tbody>
              {plainFans.map((fan) => (
                <tr key={fan.id} className="border-t border-slate-800">
                  <td className="py-0.5 text-slate-400">{fan.id}</td>
                  <td className="py-0.5 text-center">
                    <input aria-label={`fan ${fan.id} tilt`} type="number" className={num} value={fan.tilt_deg} onChange={(e) => updateFan(fan.id, { tilt_deg: clampTilt(Number(e.target.value)) })} />
                  </td>
                  <td className="py-0.5 text-center">
                    <input aria-label={`fan ${fan.id} azimuth`} type="number" className={num} value={fan.azimuth_deg} onChange={(e) => updateFan(fan.id, { azimuth_deg: wrapAzimuth(Number(e.target.value)) })} />
                  </td>
                  <td className="py-0.5 text-slate-300">{describeAxis(effectiveFan(fan, foils).axis)}</td>
                  <td className="py-0.5 text-right tabular-nums text-slate-400">{hoverU?.[scenario.fans.indexOf(fan)]?.toFixed(2) ?? "-"}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
      <FoilSheet />
      <p className="text-[10px] leading-snug text-slate-500">
        FRD body frame: X forward, Y right, Z down. Positions come from the CAD; the PX4 rotor
        geometry exported below uses the foil pressure points and the deflected directions.
      </p>
    </section>
  );
}
