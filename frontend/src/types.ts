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
  /** File name of the airframe glTF binary served by /api/cad/model/{name}. */
  cad_model?: string | null;
}

export interface FrameMapping {
  cad_forward_axis: Axis;
  cad_up_axis: Axis;
  cad_units: CadUnits;
  /** CAD coordinates (cad_units) of the FRD origin, the reference CG. */
  cad_origin?: Vec3 | null;
  /** Hover attitude of the airframe, nose-up degrees; PX4's body frame is this hover frame. */
  hover_pitch_deg?: number;
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

/**
 * A jet-deflecting foil behind a group of motors (backend/tiltlab/scenario.py Foil). The motors'
 * tilt/azimuth describe the MOTOR axis (90 / 0 = horizontal, blowing aft). The foil turns the
 * jet down by deflection_deg: 0 = straight aft (pure forward thrust), 90 = straight down (pure
 * lift), 180 = straight forward. The force acts at pressure_points_frd_m (keyed by fan id).
 */
/**
 * Coanda surface model (backend Coanda): the jet follows a convex surface of radius_m and stays
 * attached up to theta0_deg * exp(-k * jet_thickness_m / radius_m); thrust retained while attached is
 * 1 - loss_per_90deg * turning / 90, times (1 - separated_loss) once separated.
 */
export interface Coanda {
  enabled: boolean;
  radius_m: number;
  jet_thickness_m: number;
  theta0_deg: number;
  k: number;
  loss_per_90deg: number;
  separated_loss: number;
  estimated?: boolean;
  notes?: string;
}

export interface Foil {
  id: string;
  fan_ids: number[];
  deflection_deg: number;
  per_fan_deflection_deg: Record<string, number>;
  pressure_points_frd_m: Record<string, Vec3>;
  loss_at_90deg: number;
  coanda?: Coanda | null;
  estimated?: boolean;
  notes?: string;
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
  foils?: Foil[];
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
  /** Pilot-side control checks (backend metrics["control"]); absent from older fixtures. */
  control?: ControlMetrics;
  score: ScoreMetrics;
  badges: Record<string, string>;
  estimated: boolean;
  estimated_sources: string[];
  notes: string[];
}

export interface ControlAxisMetrics {
  torque_Nm: number;
  attainable: boolean;
  inertia_kgm2: number;
  /** Attainable torque over inertia, rad/s^2, at hover. */
  accel_rad_s2: number;
  linear_torque_Nm: number;
  /** Share of the torque reachable before PX4 has to desaturate a fan. */
  linear_fraction: number;
  time_to_10deg_s: number | null;
  /** Fx/Fy leaking when 20 percent of the axis is commanded, fraction of weight. */
  surge_leak_frac_of_weight: number;
  required_accel_rad_s2: number;
}

export interface ControlCheck {
  name: string;
  value: number;
  threshold: number;
  unit: string;
  pass: boolean;
}

export interface ControlMetrics {
  inertia_diag_kgm2: number[];
  inertia_placeholder: boolean;
  fan_lag_s: number;
  rate_bandwidth_rad_s: number;
  axes: Record<string, ControlAxisMetrics>;
  requirements: { min_roll_accel: number; min_pitch_accel: number; min_yaw_accel: number; max_coupling: number; max_surge_leak: number };
  checks: ControlCheck[];
  pass: boolean;
  score: number;
  weakest_axis: string | null;
}

export type MetricGroup = "hover" | "authority" | "control" | "coupling" | "conditioning" | "composite";

/** One geometry evaluated by POST /api/sweep (backend/tiltlab/core/sweep.py). */
export interface SweepCandidate {
  variable: "foil" | "tilt";
  /** Tilt variable only. */
  tilts_deg?: number[];
  azimuths_deg?: number[];
  /** Foil variable only: deflection per fan id. */
  deflections_deg?: Record<string, number>;
  left_deg?: number | null;
  right_deg?: number | null;
  /** Angle per wing pair, outer to inner (tilt or deflection depending on variable). */
  pair_tilts_deg: number[];
  centreline_tilt_deg: number;
  /** Nose fans swept sideways: signed tilt (front, rear), degrees, negative left, positive right;
   * null when the sweep left the nose fans alone. */
  nose_tilts_deg?: number[] | null;
  /** Tilt and azimuth applied to each nose fan, by fan id, degrees. */
  nose_angles_deg?: Record<string, [number, number]> | null;
  power_W: number;
  headroom: number;
  roll_Nm: number | null;
  pitch_Nm: number | null;
  yaw_Nm: number | null;
  fz_up_N: number | null;
  yaw_Nm_per_kW: number | null;
  coupling_max: number | null;
  condition_number: number | null;
  /** Control checks: angular acceleration the attainable torque gives at hover, rad/s^2. */
  roll_acc: number | null;
  pitch_acc: number | null;
  yaw_acc: number | null;
  /** Fraction of each axis' authority reachable before PX4 has to desaturate (min over axes). */
  linear_frac: number;
  /** Fore-aft/lateral force leaking out when 20 percent of an axis is commanded, fraction of weight. */
  surge_leak: number;
  control_score: number;
  weakest_axis: "roll" | "pitch" | "yaw" | null;
  hover_pitch_deg?: number;
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
  variable?: "auto" | "foil" | "tilt";
  foil_grouping?: "same" | "left_right" | "per_pair";
  azimuth_mode:
    | "inward"
    | "outward"
    | "forward"
    | "aft"
    | "alternating"
    | "outer_fwd_inner_aft"
    | "outer_aft_inner_fwd";
  per_pair: boolean;
  centreline_tilts_deg?: number[];
  /** Nose fans sideways tilt grid, signed degrees (negative left, positive right); empty = off. */
  nose_tilts_deg?: number[];
  nose_pairing?: "opposed" | "same" | "independent";
  min_headroom: number;
  min_yaw_Nm: number;
  /** rad/s^2 at hover; 0 = not filtered. */
  min_roll_accel?: number;
  min_pitch_accel?: number;
  min_yaw_accel?: number;
  /** fractions; 1 = not filtered. */
  max_coupling?: number;
  max_surge_leak?: number;
  hover_pitch_deg?: number[];
  rank_by?: "power" | "yaw" | "yaw_per_kW" | "headroom" | "score" | "control";
  top: number;
}

export interface SweepDiagnostics {
  n_controllable: number;
  n_blocked_by_thresholds: number;
  best_headroom_controllable: number | null;
  best_yaw_controllable: number | null;
  most_common_reason_near_miss: string | null;
}

export interface SweepResponse {
  variable: "foil" | "tilt";
  diagnostics: SweepDiagnostics;
  n_evaluated: number;
  n_feasible: number;
  truncated: boolean;
  elapsed_ms: number;
  objective: string;
  candidates: SweepCandidate[];
  best: SweepCandidate | null;
}

/** One row of the foil design sheet (backend/tiltlab/export/angle_sheet.py). Extra keys hold the
 * CAD coordinates, named motor_centre_cad_<units>, duct_exit_cad_<units>, pressure_point_cad_<units>. */
export interface FoilSheetRow {
  rotor: number;
  output: string;
  foil: string;
  side: "left" | "right";
  deflection_deg: number;
  change_from_as_built_deg: number;
  exhaust_angle_below_fore_aft_deg: number;
  thrust_dir_frd: string;
  thrust_dir_cad: string;
  exhaust_dir_frd: string;
  exhaust_dir_cad: string;
  pressure_point_frd_m: number[];
  thrust_scale: number;
  ct_effective_N: number;
  [cadCoordinate: string]: unknown;
}

/** What /api/gazebo/status reports: the WSL launcher session started from the app. */
export type GazeboMode = "sitl" | "hitl";
export interface GazeboStatus {
  available: boolean;
  running: boolean;
  mode: GazeboMode | null;
  harness: string | null;
  command: string | null;
  log: string | null;
  tail: string[];
  returncode: number | null;
  dry_run?: boolean;
  stopped?: boolean;
}


/** A serial port on the backend host (GET /api/board/ports); pixhawk is the USB id / name match. */
export interface SerialPortInfo {
  device: string;
  description: string;
  vid: number | null;
  pid: number | null;
  pixhawk: boolean;
}

/** GET /api/board/status: heartbeat flags and firmware of the Pixhawk on USB. */
export interface BoardStatus {
  connected: boolean;
  port: string | null;
  system_id: number | null;
  firmware: string | null;
  board_id: number | null;
  armed: boolean | null;
  hil: boolean | null;
  mode: string | null;
  ports: SerialPortInfo[];
  message: string;
}

/** POST /api/board/push: what was written through the NSH shell and what read back. */
export interface BoardPushResult {
  port: string;
  sent: number;
  changed: string[];
  verified: number;
  mismatches: { name: string; wanted: number; board: number | null }[];
  backup: string | null;
  console: string;
}
