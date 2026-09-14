import { useEffect, useState } from "react";
import { useVectraStore } from "../store";
import type { ParamInfo } from "../types";

/** Value of `name` in the previewed params lines (either "NAME\tvalue" or the QGC five-column
 * form "1\t1\tNAME\tvalue\ttype"). */
function scenarioValue(lines: string[], name: string): string | null {
  for (const ln of lines) {
    const parts = ln.split("\t");
    const i = parts.indexOf(name);
    if (i >= 0 && i + 1 < parts.length) return parts[i + 1];
  }
  return null;
}

function isInteger(info: ParamInfo | null, typeCode: number | null): boolean {
  if (info) return info.type !== "float";
  return typeCode === 6; // MAV_PARAM_TYPE_INT32
}

/** The picked parameter: catalogue entry, board value, scenario value and a write box.
 * Writes go through POST /api/board/param (shell param set, save, read back). */
export default function ParamCard() {
  const edit = useVectraStore((s) => s.paramEdit);
  const lines = useVectraStore((s) => s.paramsLines);
  const armed = useVectraStore((s) => s.board.status?.armed ?? false);
  const readParam = useVectraStore((s) => s.readParam);
  const writeParam = useVectraStore((s) => s.writeParam);
  const [draft, setDraft] = useState("");
  useEffect(() => {
    setDraft(edit.boardValue === null ? "" : String(edit.boardValue));
  }, [edit.name, edit.boardValue]);
  if (!edit.name) return null;

  const { info } = edit;
  const integer = isInteger(info, edit.typeCode);
  const choices = info?.values ?? (info?.type === "boolean" ? { "0": "Disabled", "1": "Enabled" } : null);
  const parsed = draft.trim() === "" ? NaN : Number(draft);
  const inRange = (info?.min == null || parsed >= info.min) && (info?.max == null || parsed <= info.max);
  const valid = Number.isFinite(parsed) && (!integer || Number.isInteger(parsed)) && inRange;
  const unchanged = edit.boardValue !== null && parsed === edit.boardValue;
  const inScenario = scenarioValue(lines, edit.name);
  const range = info && (info.min !== null || info.max !== null) ? `${info.min ?? ""} to ${info.max ?? ""}` : null;
  const meta = info ? [info.group, info.type, info.unit].filter(Boolean).join(" · ") : "not in the pinned tree's definitions";

  return (
    <div className="flex flex-col gap-1 rounded border border-slate-700 bg-slate-950/60 p-2 text-xs" data-testid="param-card">
      <div className="flex items-baseline justify-between gap-2">
        <span className="font-mono text-slate-100">{edit.name}</span>
        <span className="text-right text-[10px] text-slate-500">{meta}</span>
      </div>
      {info && <p className="text-slate-300">{info.short}</p>}
      {info?.long && <p className="whitespace-pre-line text-[10px] text-slate-500">{info.long}</p>}
      <div className="grid grid-cols-[auto_1fr_auto] items-center gap-x-2 gap-y-1">
        <span className="text-slate-400">board</span>
        <span className="font-mono tabular-nums" data-testid="board-value">
          {edit.boardValue === null ? "unread" : edit.boardValue}
        </span>
        <button className="ui-btn" onClick={() => void readParam()} disabled={edit.busy}>
          read
        </button>
        {inScenario !== null && (
          <>
            <span className="text-slate-400">scenario</span>
            <span className="font-mono tabular-nums">{inScenario}</span>
            <span />
          </>
        )}
        {range && (
          <>
            <span className="text-slate-400">range</span>
            <span className="tabular-nums">
              {range}
              {info?.default != null ? `, default ${info.default}` : ""}
            </span>
            <span />
          </>
        )}
        <label className="text-slate-400" htmlFor="param-new-value">
          new
        </label>
        {choices ? (
          <select id="param-new-value" value={draft} onChange={(e) => setDraft(e.target.value)} disabled={edit.busy}>
            {draft !== "" && !(draft in choices) && <option value={draft}>{draft}</option>}
            {Object.entries(choices).map(([code, label]) => (
              <option key={code} value={code}>
                {code} {label}
              </option>
            ))}
          </select>
        ) : (
          <input
            id="param-new-value"
            type="number"
            className="ui-input w-full font-mono"
            step={integer ? 1 : "any"}
            value={draft}
            onChange={(e) => setDraft(e.target.value)}
            disabled={edit.busy}
          />
        )}
        <button
          className="ui-btn-primary"
          onClick={() => void writeParam(parsed)}
          disabled={edit.busy || !valid || unchanged || armed}
          title={armed ? "the board is armed" : "param set, param save, read back"}
        >
          write
        </button>
      </div>
      {edit.message && (
        <p className="text-[10px] text-slate-400" data-testid="param-message">
          {edit.message}
        </p>
      )}
      {armed && <p className="text-[10px] text-rose-300">the board is armed: no writes</p>}
    </div>
  );
}
