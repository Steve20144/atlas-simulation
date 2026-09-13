import { describeAxis, effectiveFan } from "../geometry";
import { useTiltlabStore } from "../store";
import type { Fan, Foil } from "../types";
import FanAngleInputs, { ANGLE_HEADINGS, AngleModeToggle } from "./FanAngleInputs";

const num = "w-16 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-right tabular-nums";

/** The fans that do not blow into a foil (nose fans): angles, resulting thrust direction, hover u. */
export default function CentrelineFans({ fans, foils }: { fans: Fan[]; foils: Foil[] }) {
  const allFans = useTiltlabStore((s) => s.scenario.fans);
  const hoverU = useTiltlabStore((s) => s.metrics?.hover?.u);
  const mode = useTiltlabStore((s) => s.angleMode);
  const [h1, h2] = ANGLE_HEADINGS[mode];
  return (
    <div className="ui-sub">
      <div className="flex items-center justify-between">
        <h3>Centreline fans</h3>
        <AngleModeToggle />
      </div>
      <table className="mt-1 w-full text-[11px]">
        <thead>
          <tr className="text-slate-400">
            <th className="text-left">fan</th>
            <th>{h1}</th>
            <th>{h2}</th>
            <th className="text-left">thrust</th>
            <th title="hover command 0..1">u</th>
          </tr>
        </thead>
        <tbody>
          {fans.map((fan) => (
            <tr key={fan.id} className="border-t border-slate-800">
              <td className="py-0.5 text-slate-400">{fan.id}</td>
              <FanAngleInputs fan={fan} className={num} />
              <td className="py-0.5 text-slate-300">{describeAxis(effectiveFan(fan, foils).axis)}</td>
              <td className="py-0.5 text-right tabular-nums text-slate-400">
                {hoverU?.[allFans.indexOf(fan)]?.toFixed(2) ?? "-"}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      <p className="mt-1 text-[10px] leading-snug text-slate-500">
        {mode === "fwd_side"
          ? "Forward tilt leans the thrust forward (+) or aft (-); side tilt leans it right (+) or left (-). Each is the angle from vertical seen in its own plane, as on the printed mount."
          : "Tilt is the angle from vertical; azimuth is the direction the fan leans towards (0 forward, 90 right)."}
      </p>
    </div>
  );
}
