/**
 * Scenario data model, mirroring PLAN.md section 5 (scenarios/*.json).
 * Frames: FRD body, NED world. SI units internally; degrees only for tilt,
 * azimuth and attitude offsets, exactly as they appear in the stored JSON.
 */

export type Vec3 = [number, number, number];
export type Mat3 = [Vec3, Vec3, Vec3];

export type Axis = "+X" | "-X" | "+Y" | "-Y" | "+Z" | "-Z";
export type CadUnits = "mm" | "m" | "in";
export type SpinDirection = "CW" | "CCW";
export type MassSource = "user" | "density" | "cad";
export type ControlConcept = "stock" | "fully_actuated";

export interface ScenarioMeta {
  name: string;
  created: string;
  px4_version: string;
}

export interface FrameMapping {
  cad_forward_axis: Axis;
  cad_up_axis: Axis;
  cad_units: CadUnits;
}

export interface CadReportedMass {
  mass_kg: number;
  cg_m: Vec3;
}

export interface MassBody {
  name: string;
  volume_m3: number;
  mass_kg: number;
  source: MassSource;
}

export interface MassProperties {
  total_kg: number;
  cg_frd_m: Vec3;
  inertia_frd_kgm2: Mat3;
  cad_reported: CadReportedMass;
  bodies: MassBody[];
}

export interface Fan {
  id: number;
  output: string;
  pos_frd_m: Vec3;
  tilt_deg: number;
  azimuth_deg: number;
  spin: SpinDirection;
  mirror_of: number | null;
  curve_ref: string;
}

export interface FanCurvePoint {
  cmd: number;
  thrust_N: number;
  power_W: number;
}

export interface FanCurve {
  cells: number;
  points: FanCurvePoint[];
  lag_s: number;
  max_continuous_A: number;
  notes: string;
}

export interface ControlSettings {
  concept: ControlConcept;
  blend: number;
  ca_method: number;
  px4_params_override: Record<string, number>;
}

export interface RigSettings {
  enabled: boolean;
  lock_position: boolean;
  lock_roll: boolean;
  lock_pitch: boolean;
  lock_yaw: boolean;
  attitude_offset_deg: Vec3;
}

export interface EnvironmentSettings {
  air_density: number;
  wind_ned_mps: Vec3;
  gravity: number;
}

export interface OutputSelection {
  params: boolean;
  csv: boolean;
  report: boolean;
  plots: boolean;
  sdf: boolean;
  angle_sheet: boolean;
}

export interface Scenario {
  meta: ScenarioMeta;
  frame: FrameMapping;
  mass: MassProperties;
  fans: Fan[];
  fan_curves: Record<string, FanCurve>;
  control: ControlSettings;
  rig: RigSettings;
  environment: EnvironmentSettings;
  outputs: OutputSelection;
}
