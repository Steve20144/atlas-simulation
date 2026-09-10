/**
 * Scenario data model, mirroring PLAN.md section 5 (scenarios/*.json) and
 * backend/tiltlab/scenario.py. Frames: FRD body, NED world. SI units
 * internally; degrees only for tilt, azimuth and attitude offsets, exactly as
 * they appear in the stored JSON.
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
  notes?: string;
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
  cad_reported: CadReportedMass | null;
  bodies: MassBody[];
  estimated?: boolean;
  notes?: string;
}

export interface Fan {
  id: number;
  output: string;
  /** Thrust application point, metres, FRD body frame. */
  pos_frd_m: Vec3;
  tilt_deg: number;
  azimuth_deg: number;
  spin: SpinDirection;
  mirror_of: number | null;
  curve_ref: string;
  /** KM magnitude (N m per N); sign comes from spin. */
  km?: number;
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
  estimated?: boolean;
}

export interface ControlSettings {
  concept: ControlConcept;
  blend: number;
  ca_method: number;
  reaction_torque?: boolean;
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

/* ---------- Metrics returned by POST /api/metrics (M4 contract) ---------- */

export interface HoverMetrics {
  /** Normalised actuator commands 0..1, one per fan. */
  u: number[];
  /** Thrust per fan in N. */
  thrust_N: number[];
  power_W: number;
  headroom: number;
}

/** One controlled axis: attainable authority plus/minus in `unit` (N m or N) at the trim. */
export interface AuthorityAxis {
  plus: number;
  minus: number;
  unit: string;
  plus_per_W: number | null;
  minus_per_W: number | null;
  plus_attainable?: boolean;
  minus_attainable?: boolean;
  /** "stock_px4" or "needs_fully_actuated_controller". */
  badge: string;
}

/** Backend axis names (core/metrics.py AXIS_NAMES): forces are capitalised. */
export type AuthorityKey = "roll" | "pitch" | "yaw" | "Fx" | "Fy" | "Fz";
export type AuthorityMetrics = Partial<Record<AuthorityKey, AuthorityAxis>>;

/** Leakage matrix: rows are commanded controlled axes, columns the controlled axes. */
export interface CouplingMetrics {
  axes: string[];
  leakage_fraction: number[][];
  max_offaxis_fraction?: number;
  badge?: string;
}

export interface ConditioningMetrics {
  axes: string[];
  singular_values: number[];
  rank: number;
  null_space_dim: number;
  condition_number?: number;
}

export interface ScoreMetrics {
  value: number;
  weights: Record<string, number>;
  normalised?: Record<string, number>;
}

/** POST /api/metrics response (backend/tiltlab/api/schemas.py MetricsResponse). */
export interface Metrics {
  scenario_name: string;
  concept: ControlConcept;
  controlled_axes: string[];
  collective: number;
  /** Collective (fraction of Fz_max) at which the backend's hover solution sits. */
  collective_hover?: number;
  hover: HoverMetrics;
  authority: AuthorityMetrics;
  coupling: CouplingMetrics;
  conditioning: ConditioningMetrics;
  score: ScoreMetrics;
  badges: Record<string, string>;
  estimated: boolean;
  estimated_sources: string[];
  notes: string[];
}

export type MetricGroup = "hover" | "authority" | "coupling" | "conditioning" | "composite";

/** One geometry evaluated by POST /api/sweep (backend/tiltlab/core/sweep.py). */
export interface SweepCandidate {
  tilts_deg: number[];
  azimuths_deg: number[];
  pair_tilts_deg: number[];
  centreline_tilt_deg: number;
  power_W: number;
  headroom: number;
  roll_Nm: number | null;
  pitch_Nm: number | null;
  yaw_Nm: number | null;
  fz_up_N: number | null;
  yaw_Nm_per_kW: number | null;
  coupling_max: number | null;
  condition_number: number | null;
  score: number;
  estimated: boolean;
  feasible: boolean;
  reasons: string[];
}

export interface SweepRequestBody {
  scenario: Scenario;
  concept: ControlConcept;
  collective?: number;
  tilts_deg: number[];
  azimuth_mode: "inward" | "outward" | "forward" | "aft";
  per_pair: boolean;
  centreline_tilts_deg?: number[];
  min_headroom: number;
  min_yaw_Nm: number;
  top: number;
}

export interface SweepResponse {
  n_evaluated: number;
  n_feasible: number;
  truncated: boolean;
  elapsed_ms: number;
  objective: string;
  candidates: SweepCandidate[];
  best: SweepCandidate | null;
}
