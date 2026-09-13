import { azimuthNote } from "../geometry";
import { useTiltlabStore } from "../store";
import FanRow from "./FanRow";

const th = "px-1 py-1 text-left text-xs font-medium text-slate-400";

/** Left panel: 10 fan rows with output, tilt, azimuth, mirror lock and spin. */
export default function FanTable() {
  const fans = useTiltlabStore((s) => s.scenario.fans);
  const hoverU = useTiltlabStore((s) => s.metrics?.hover?.u);
  const massKg = useTiltlabStore((s) => s.scenario.mass.total_kg);

  return (
    <section className="ui-card flex h-full flex-col overflow-auto">
      <div className="mb-1 flex items-baseline justify-between">
        <h2>Fans</h2>
        <span className="text-xs text-slate-400">mass {massKg.toFixed(2)} kg</span>
      </div>
      {fans.map(azimuthNote).filter(Boolean).map((n) => (
        <p key={n} className="mb-1 text-[10px]" style={{ color: "var(--ui-warn)" }} role="note">
          {n}
        </p>
      ))}
      {fans.length === 0 ? (
        <p className="text-xs text-slate-500">No scenario loaded. Pick one in the top bar.</p>
      ) : (
        <table className="w-full text-xs">
          <thead>
            <tr>
              <th className={th}>id</th>
              <th className={th}>output</th>
              <th className={th}>tilt deg</th>
              <th className={th}>azimuth deg</th>
              <th className={th} title="Editing a fan also edits its partner with azimuth 360 - az">
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
        FRD body frame. Tilt 0 points thrust up (-Z); azimuth 0 tilts forward, 90 right. Arrow keys
        nudge by 1 degree.
      </p>
    </section>
  );
}
