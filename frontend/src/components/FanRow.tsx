import type { KeyboardEvent } from "react";
import { clampTilt, wrapAzimuth } from "../geometry";
import { useTiltlabStore } from "../store";
import type { Fan, SpinDirection } from "../types";

const cell = "w-16 rounded border border-slate-600 bg-slate-800 px-1 py-0.5 text-right tabular-nums";

interface Props {
  fan: Fan;
  hoverU?: number;
}

/** One row of the fan table. Arrow up/down nudge tilt and azimuth by 1 degree. */
export default function FanRow({ fan, hoverU }: Props) {
  const locked = useTiltlabStore((s) => s.mirrorLock[fan.id] ?? false);
  const { updateFan, setMirrorLock } = useTiltlabStore.getState();

  const nudge = (field: "tilt_deg" | "azimuth_deg") => (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key !== "ArrowUp" && e.key !== "ArrowDown") return;
    e.preventDefault();
    const delta = e.key === "ArrowUp" ? 1 : -1;
    // Read the live value so rapid key repeats never nudge from a stale render.
    const live = useTiltlabStore.getState().scenario.fans.find((f) => f.id === fan.id) ?? fan;
    const next = live[field] + delta;
    updateFan(fan.id, { [field]: field === "tilt_deg" ? clampTilt(next) : wrapAzimuth(next) });
  };

  return (
    <tr className="border-t border-slate-800" data-testid={`fan-row-${fan.id}`}>
      <td className="px-1 py-0.5 tabular-nums text-slate-400">{fan.id}</td>
      <td className="px-1 py-0.5">
        <input
          aria-label={`Output fan ${fan.id}`}
          className="w-16 rounded border border-slate-600 bg-slate-800 px-1 py-0.5"
          value={fan.output}
          onChange={(e) => updateFan(fan.id, { output: e.target.value })}
        />
      </td>
      <td className="px-1 py-0.5">
        <input
          aria-label={`Tilt fan ${fan.id}`}
          className={cell}
          type="number"
          min={0}
          max={90}
          step={1}
          value={fan.tilt_deg}
          onKeyDown={nudge("tilt_deg")}
          onChange={(e) => updateFan(fan.id, { tilt_deg: clampTilt(Number(e.target.value)) })}
        />
      </td>
      <td className="px-1 py-0.5">
        <input
          aria-label={`Azimuth fan ${fan.id}`}
          className={fan.tilt_deg === 0 ? `${cell} opacity-50` : cell}
          title={fan.tilt_deg === 0 ? "no effect while tilt is 0: the fan points straight up; tilt it and azimuth picks the direction (0 forward, 90 right)" : "0 forward, 90 right, 180 aft, 270 left"}
          type="number"
          min={0}
          max={360}
          step={1}
          value={fan.azimuth_deg}
          onKeyDown={nudge("azimuth_deg")}
          onChange={(e) => updateFan(fan.id, { azimuth_deg: wrapAzimuth(Number(e.target.value)) })}
        />
      </td>
      <td className="px-1 py-0.5 text-center">
        <input
          aria-label={`Mirror lock fan ${fan.id}`}
          type="checkbox"
          disabled={fan.mirror_of === null}
          title={fan.mirror_of === null ? "no mirror partner" : `partner fan ${fan.mirror_of}`}
          checked={locked}
          onChange={(e) => setMirrorLock(fan.id, e.target.checked)}
        />
      </td>
      <td className="px-1 py-0.5">
        <select
          aria-label={`Spin fan ${fan.id}`}
          className="rounded border border-slate-600 bg-slate-800 px-1 py-0.5"
          value={fan.spin}
          onChange={(e) => updateFan(fan.id, { spin: e.target.value as SpinDirection })}
        >
          <option value="CW">CW</option>
          <option value="CCW">CCW</option>
        </select>
      </td>
      <td className="px-1 py-0.5 text-right tabular-nums text-slate-400">
        {hoverU === undefined ? "-" : hoverU.toFixed(2)}
      </td>
    </tr>
  );
}
