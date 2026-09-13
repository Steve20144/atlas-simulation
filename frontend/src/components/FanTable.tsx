import { useTiltlabStore } from "../store";
import { ANGLE_HEADINGS, AngleModeToggle } from "./FanAngleInputs";
import FanRow from "./FanRow";

const th = "px-1 py-1 text-left text-xs font-medium text-slate-400";

/** Left panel: 10 fan rows with output, tilt, azimuth, mirror lock and spin. */
export default function FanTable() {
  const fans = useTiltlabStore((s) => s.scenario.fans);
  const hoverU = useTiltlabStore((s) => s.metrics?.hover?.u);
  const massKg = useTiltlabStore((s) => s.scenario.mass.total_kg);
  const mode = useTiltlabStore((s) => s.angleMode);
  const [h1, h2] = ANGLE_HEADINGS[mode];

  return (
    <section className="ui-card flex h-full flex-col overflow-auto">
      <div className="mb-1 flex items-baseline justify-between">
        <h2>Fans</h2>
        <span className="text-xs text-slate-400">mass {massKg.toFixed(2)} kg</span>
      </div>
      <div className="mb-1 flex items-center justify-between">
        <span className="ui-label">angles</span>
        <AngleModeToggle />
      </div>
      {fans.length === 0 ? (
        <p className="text-xs text-slate-500">No scenario loaded. Pick one in the top bar.</p>
      ) : (
        <table className="w-full text-xs">
          <thead>
            <tr>
              <th className={th}>id</th>
              <th className={th}>output</th>
              <th className={th}>{h1}</th>
              <th className={th}>{h2}</th>
              <th className={th} title="Editing a fan also edits its partner mirrored left/right">
                mirror
              </th>
              <th className={th}>spin</th>
              <th className={`${th} text-right`} title="hover command 0..1">
                u
              </th>
            </tr>
          </thead>
          <tbody>
            {fans.map((f, i) => (
              <FanRow key={f.id} fan={f} hoverU={hoverU?.[i]} />
            ))}
          </tbody>
        </table>
      )}
      <p className="mt-2 text-[10px] leading-snug text-slate-500">
        {mode === "fwd_side"
          ? "FRD body frame. Forward tilt leans the thrust forward (+) or aft (-), side tilt leans it right (+) or left (-), each measured from vertical in its own plane. Arrow keys nudge by 1 degree."
          : "FRD body frame. Tilt 0 points thrust up (-Z); azimuth 0 tilts forward, 90 right. Arrow keys nudge by 1 degree."}
      </p>
    </section>
  );
}
