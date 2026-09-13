import { useTiltlabStore } from "../store";
import type { Fan, SpinDirection } from "../types";
import FanAngleInputs from "./FanAngleInputs";

interface Props {
  fan: Fan;
  hoverU?: number;
}

/** One row of the fan table: output, the two angles (FanAngleInputs), mirror lock, spin, hover u. */
export default function FanRow({ fan, hoverU }: Props) {
  const locked = useTiltlabStore((s) => s.mirrorLock[fan.id] ?? false);
  const { updateFan, setMirrorLock } = useTiltlabStore.getState();

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
      <FanAngleInputs fan={fan} />
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
