import { describeAxis, effectiveFan } from "../geometry";
import { useTiltlabStore } from "../store";
import type { Foil } from "../types";

const num = "w-16 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-right tabular-nums";

interface Props {
  foil: Foil;
  title: string;
}

/** One foil: a deflection slider, the resulting thrust direction in words, optional per-motor angles. */
export default function FoilCard({ foil, title }: Props) {
  const fans = useTiltlabStore((s) => s.scenario.fans);
  const foils = useTiltlabStore((s) => s.scenario.foils);
  const hoverU = useTiltlabStore((s) => s.metrics?.hover?.u);
  const { setFoilDeflection, setFoilFanDeflection } = useTiltlabStore.getState();
  const segmented = Object.keys(foil.per_fan_deflection_deg ?? {}).length > 0;
  const first = fans.find((f) => f.id === foil.fan_ids[0]);
  const eff = first ? effectiveFan(first, foils) : null;

  return (
    <div className="ui-sub" data-testid={`foil-${foil.id}`}>
      <div className="flex items-baseline justify-between">
        <h3>{title}</h3>
        <span className="text-[10px] text-slate-400">motors {foil.fan_ids.join(", ")}</span>
      </div>
      <label className="mt-1 flex items-center gap-2 text-xs">
        <span className="w-16">deflection</span>
        <input
          aria-label={`${title} deflection`}
          type="range"
          min={0}
          max={180}
          step={1}
          className="flex-1"
          value={foil.deflection_deg}
          onChange={(e) => setFoilDeflection(foil.id, Number(e.target.value))}
        />
        <input
          aria-label={`${title} deflection degrees`}
          type="number"
          min={0}
          max={180}
          className={num}
          value={foil.deflection_deg}
          onChange={(e) => setFoilDeflection(foil.id, Math.min(180, Math.max(0, Number(e.target.value))))}
        />
        <span className="text-slate-400">deg</span>
      </label>
      {eff && !segmented && (
        <p className={`mt-1 text-[11px] ${eff.attached ? "text-emerald-300" : "text-rose-300"}`}>
          thrust {describeAxis(eff.axis)}
          {eff.coandaLimitDeg !== null &&
            (eff.attached
              ? ` · jet attached (Coanda limit ${eff.coandaLimitDeg.toFixed(0)} deg), thrust kept ${(eff.ctScale * 100).toFixed(0)}%`
              : ` · jet SEPARATES: the foil asks ${eff.deflectionDeg?.toFixed(0)} deg but the jet leaves at ${eff.coandaLimitDeg.toFixed(0)} deg, thrust kept ${(eff.ctScale * 100).toFixed(0)}%`)}
        </p>
      )}
      <details className="mt-1 text-[11px]" open={segmented}>
        <summary className="cursor-pointer text-slate-400">
          per-motor angles {segmented ? "(segmented foil)" : "(same for all motors)"}
        </summary>
        <table className="mt-1 w-full">
          <tbody>
            {foil.fan_ids.map((id) => {
              const fan = fans.find((f) => f.id === id);
              if (!fan) return null;
              const e = effectiveFan(fan, foils);
              const override = foil.per_fan_deflection_deg?.[String(id)];
              return (
                <tr key={id} className="border-t border-slate-800">
                  <td className="py-0.5 text-slate-400">motor {id}</td>
                  <td className="py-0.5">
                    <input
                      aria-label={`motor ${id} deflection`}
                      type="number"
                      min={0}
                      max={180}
                      className={num}
                      value={override ?? foil.deflection_deg}
                      onChange={(ev) => setFoilFanDeflection(foil.id, id, Math.min(180, Math.max(0, Number(ev.target.value))))}
                    />
                    {override !== undefined && (
                      <button className="ml-1 text-slate-400 hover:text-slate-200" title="use the foil angle" onClick={() => setFoilFanDeflection(foil.id, id, null)}>
                        reset
                      </button>
                    )}
                  </td>
                  <td className={`py-0.5 ${e.attached ? "text-slate-300" : "text-rose-300"}`}>
                    {describeAxis(e.axis)}
                    {!e.attached && " (separated)"}
                  </td>
                  <td className="py-0.5 text-right tabular-nums text-slate-400" title="hover command 0..1">
                    {hoverU?.[fans.indexOf(fan)]?.toFixed(2) ?? "-"}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </details>
    </div>
  );
}
