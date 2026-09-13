import type { KeyboardEvent } from "react";
import { clampSideTilt, fwdSideToTiltAzimuth, pastHorizontal, signedTilt, tiltAzimuthToFwdSide, wrapAzimuth } from "../geometry";
import { useTiltlabStore, type AngleMode } from "../store";
import type { Fan } from "../types";

const cell = "w-16 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-right tabular-nums";

/** Column headings for the two angle inputs in each mode. */
export const ANGLE_HEADINGS: Record<AngleMode, [string, string]> = {
  fwd_side: ["fwd tilt deg", "side tilt deg"],
  tilt_azimuth: ["tilt deg", "azimuth deg"],
};

const round1 = (v: number) => Math.round(v * 10) / 10;

/**
 * The two angle inputs of one fan, in the mode picked in the store. Forward/side tilt are two
 * independent angles (thrust leaning forward or aft, and right or left), the way a mount is
 * measured and printed; tilt/azimuth is PX4's one tilt plus a compass direction. Both edit the
 * same scenario fields, so the export never changes. Arrow up/down nudge by 1 degree.
 */
export default function FanAngleInputs({ fan, className = cell }: { fan: Fan; className?: string }) {
  const mode = useTiltlabStore((s) => s.angleMode);
  const updateFan = useTiltlabStore((s) => s.updateFan);
  const live = () => useTiltlabStore.getState().scenario.fans.find((f) => f.id === fan.id) ?? fan;

  if (mode === "tilt_azimuth") {
    const nudge = (field: "tilt_deg" | "azimuth_deg") => (e: KeyboardEvent<HTMLInputElement>) => {
      if (e.key !== "ArrowUp" && e.key !== "ArrowDown") return;
      e.preventDefault();
      const cur = live();
      const next = cur[field] + (e.key === "ArrowUp" ? 1 : -1);
      if (field === "tilt_deg") {
        const { tilt, azimuth } = signedTilt(next, cur.azimuth_deg);
        updateFan(fan.id, { tilt_deg: tilt, azimuth_deg: azimuth });
      } else {
        updateFan(fan.id, { azimuth_deg: wrapAzimuth(next) });
      }
    };
    return (
      <>
        <td className="px-1 py-0.5">
          <input aria-label={`Tilt fan ${fan.id}`} className={className} type="number" min={-180} max={180} step={1}
            title="0 up, 90 horizontal, up to 180 pointing down; a negative value leans the opposite way (turns the azimuth by 180)"
            value={fan.tilt_deg} onKeyDown={nudge("tilt_deg")}
            onChange={(e) => {
              const { tilt, azimuth } = signedTilt(Number(e.target.value), fan.azimuth_deg);
              updateFan(fan.id, { tilt_deg: tilt, azimuth_deg: azimuth });
            }} />
        </td>
        <td className="px-1 py-0.5">
          <input aria-label={`Azimuth fan ${fan.id}`} className={fan.tilt_deg === 0 ? `${className} opacity-50` : className}
            title={fan.tilt_deg === 0 ? "no effect while tilt is 0; switch to fwd/side angles to lean the fan directly" : "0 forward, 90 right, 180 aft, 270 left"}
            type="number" min={0} max={360} step={1} value={fan.azimuth_deg} onKeyDown={nudge("azimuth_deg")}
            onChange={(e) => updateFan(fan.id, { azimuth_deg: wrapAzimuth(Number(e.target.value)) })} />
        </td>
      </>
    );
  }

  if (pastHorizontal(fan.tilt_deg)) {
    return (
      <td className="px-1 py-0.5 text-[10px]" colSpan={2} style={{ color: "var(--ui-warn)" }} role="note">
        tilt {fan.tilt_deg} az {fan.azimuth_deg}: past horizontal, edit as tilt / azimuth
      </td>
    );
  }
  const { fwd, side } = tiltAzimuthToFwdSide(fan.tilt_deg, fan.azimuth_deg);
  const setFwdSide = (f: number, s: number) => {
    const { tilt, azimuth } = fwdSideToTiltAzimuth(clampSideTilt(f), clampSideTilt(s));
    updateFan(fan.id, { tilt_deg: tilt, azimuth_deg: azimuth });
  };
  const nudge = (which: "fwd" | "side") => (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key !== "ArrowUp" && e.key !== "ArrowDown") return;
    e.preventDefault();
    const cur = tiltAzimuthToFwdSide(live().tilt_deg, live().azimuth_deg);
    const d = e.key === "ArrowUp" ? 1 : -1;
    setFwdSide(round1(cur.fwd) + (which === "fwd" ? d : 0), round1(cur.side) + (which === "side" ? d : 0));
  };
  return (
    <>
      <td className="px-1 py-0.5">
        <input aria-label={`Forward tilt fan ${fan.id}`} className={className} type="number" min={-89} max={89} step={1}
          title="thrust leans forward (+) or aft (-), degrees from vertical seen from the side"
          value={round1(fwd)} onKeyDown={nudge("fwd")}
          onChange={(e) => setFwdSide(Number(e.target.value), side)} />
      </td>
      <td className="px-1 py-0.5">
        <input aria-label={`Side tilt fan ${fan.id}`} className={className} type="number" min={-89} max={89} step={1}
          title="thrust leans right (+) or left (-), degrees from vertical seen from the front"
          value={round1(side)} onKeyDown={nudge("side")}
          onChange={(e) => setFwdSide(fwd, Number(e.target.value))} />
      </td>
    </>
  );
}

/** Two small tabs picking how fan angles are entered; state lives in the store. */
export function AngleModeToggle() {
  const mode = useTiltlabStore((s) => s.angleMode);
  const setAngleMode = useTiltlabStore((s) => s.setAngleMode);
  return (
    <span className="ui-tabs" role="group" aria-label="Angle mode">
      <button className="ui-tab" aria-pressed={mode === "fwd_side"} onClick={() => setAngleMode("fwd_side")}
        title="two independent angles: forward/aft lean and right/left lean">
        fwd / side
      </button>
      <button className="ui-tab" aria-pressed={mode === "tilt_azimuth"} onClick={() => setAngleMode("tilt_azimuth")}
        title="PX4 style: one tilt from vertical plus the compass direction it leans towards">
        tilt / azimuth
      </button>
    </span>
  );
}
