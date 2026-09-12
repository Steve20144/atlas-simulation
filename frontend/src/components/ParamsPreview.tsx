import { useState } from "react";
import { useTiltlabStore } from "../store";

/** PX4 .params preview (CA_ROTORn_* lines from the backend) with a copy button. */
export default function ParamsPreview() {
  const lines = useTiltlabStore((s) => s.paramsLines);
  const [copied, setCopied] = useState(false);
  const text = lines.join("\n");

  const copy = async () => {
    try {
      await navigator.clipboard.writeText(text);
      setCopied(true);
      setTimeout(() => setCopied(false), 1500);
    } catch {
      setCopied(false);
    }
  };

  return (
    <div className="flex flex-col gap-1">
      <div className="flex items-center justify-between">
        <h3>PX4 params preview</h3>
        <button
          className="ui-btn"
          onClick={() => void copy()}
          disabled={lines.length === 0}
        >
          {copied ? "copied" : "copy"}
        </button>
      </div>
      <textarea
        aria-label="PX4 params preview"
        readOnly
        spellCheck={false}
        className="h-48 w-full resize-y rounded border border-slate-700 bg-slate-950 p-2 font-mono text-[11px] leading-tight text-slate-200"
        value={text}
        placeholder="CA_ROTORn_* lines appear here once a scenario is loaded"
      />
    </div>
  );
}
