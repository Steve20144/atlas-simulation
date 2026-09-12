import { useTiltlabStore } from "../store";

const btn = "rounded border border-slate-600 px-2 py-1 text-xs hover:bg-slate-700";
const btnActive = "rounded border border-sky-400 bg-sky-900 px-2 py-1 text-xs";
const QUICK = [0, 15, 25];

/**
 * Hover attitude of the airframe in nose-up degrees (scenario.frame.hover_pitch_deg). PX4's body
 * frame is this hover frame: the exported CA_ROTOR geometry, the Gazebo model and every trim and
 * authority number are computed for the airframe sitting at this pitch, so it belongs next to the
 * collective, not buried in a sweep. Backend bounds: -90 to 90.
 */
export default function HoverPitchControl() {
  const pitch = useTiltlabStore((s) => s.scenario.frame.hover_pitch_deg ?? 0);
  const setHoverPitch = useTiltlabStore((s) => s.setHoverPitch);
  return (
    <label
      className="flex items-center gap-2 text-xs"
      title="Nose-up hover attitude of the airframe. PX4 holds 'level' in this frame, so 25 means the aircraft hovers 25 degrees nose-up. Changes the trim, the authority numbers and both exports."
    >
      Hover pitch
      <input
        aria-label="Hover pitch"
        type="number"
        min={-90}
        max={90}
        step={1}
        value={pitch}
        className="w-16 rounded border border-slate-600 bg-slate-800 px-2 py-1 tabular-nums"
        onChange={(e) => {
          const v = Number(e.target.value);
          if (Number.isFinite(v)) setHoverPitch(v);
        }}
      />
      <span className="text-slate-400">deg</span>
      <span className="flex gap-1" role="group" aria-label="Hover pitch presets">
        {QUICK.map((q) => (
          <button key={q} className={q === pitch ? btnActive : btn} onClick={() => setHoverPitch(q)}>
            {q}
          </button>
        ))}
      </span>
    </label>
  );
}
