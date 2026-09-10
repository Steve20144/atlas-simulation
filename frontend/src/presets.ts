import type { Fan, Scenario } from "./types";

export type PresetId = "dihedral30" | "vertical" | "omni";

export const PRESETS: { id: PresetId; label: string; title: string }[] = [
  { id: "dihedral30", label: "Dihedral 30", title: "Load scenarios/baseline_dihedral30.json" },
  { id: "vertical", label: "All vertical", title: "Tilt 0 on all fans" },
  { id: "omni", label: "Omni-style", title: "Tilt 45, azimuths alternating 0/90/180/270" },
];

/** Name of the stored scenario backing the dihedral preset (GET /api/scenarios/{name}). */
export const DIHEDRAL_SCENARIO_NAME = "baseline_dihedral30";

function withFans(scenario: Scenario, map: (fan: Fan, index: number) => Fan): Scenario {
  return { ...scenario, fans: scenario.fans.map(map) };
}

/** All fans vertical: tilt 0 (azimuth is irrelevant at tilt 0, reset to 0 for clarity). */
export function presetVertical(scenario: Scenario): Scenario {
  return withFans(scenario, (f) => ({ ...f, tilt_deg: 0, azimuth_deg: 0 }));
}

/** Omni-style: tilt 45 degrees, azimuth cycling 0, 90, 180, 270 by fan index. */
export function presetOmni(scenario: Scenario): Scenario {
  return withFans(scenario, (f, i) => ({ ...f, tilt_deg: 45, azimuth_deg: (i % 4) * 90 }));
}
