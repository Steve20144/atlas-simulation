import { useTiltlabStore } from "../store";
import FanTable from "./FanTable";
import FoilPanel from "./FoilPanel";

/** Left column: foil controls when the scenario has foils, otherwise the raw per-fan table. */
export default function GeometryPanel() {
  const hasFoils = useTiltlabStore((s) => (s.scenario.foils?.length ?? 0) > 0);
  return hasFoils ? <FoilPanel /> : <FanTable />;
}
