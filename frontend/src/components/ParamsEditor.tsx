import { useVectraStore } from "../store";
import ParamCard from "./ParamCard";
import ParamSearch from "./ParamSearch";
import ParamsPreview from "./ParamsPreview";

/** PX4 params: search any parameter, see what the board and the scenario have, write a new
 * value. The scenario's params file preview (copyable) sits underneath, folded. */
export default function ParamsEditor() {
  const selectParam = useVectraStore((s) => s.selectParam);
  const clearParam = useVectraStore((s) => s.clearParam);
  const name = useVectraStore((s) => s.paramEdit.name);
  const lines = useVectraStore((s) => s.paramsLines);

  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center justify-between">
        <h3>PX4 params</h3>
        {name && (
          <button className="ui-btn" onClick={clearParam}>
            clear
          </button>
        )}
      </div>
      <ParamSearch onPick={(n) => void selectParam(n)} />
      <ParamCard />
      <details className="text-xs">
        <summary className="cursor-pointer text-slate-400">scenario params file ({lines.length} lines)</summary>
        <div className="mt-1">
          <ParamsPreview />
        </div>
      </details>
    </div>
  );
}
