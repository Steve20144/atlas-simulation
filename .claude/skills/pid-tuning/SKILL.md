---
name: pid-tuning
description: Tune the PX4 rate, attitude and position gains of the Atlas 10-EDF aircraft in SITL, one change per flight, from a Vectra scenario to a set of gains baked into the scenario. Covers the sizing rule, the run-compare-change loop with hover_run.sh and tune_table.py, the allocator residual trap that freezes the integrators, and how to read ringing, drift and wander. Use when the user asks to tune PID or gains, says the aircraft wobbles, drifts, yaws or will not hold position in the sim, or wants gains carried to the Pixhawk. Needs a running SITL: see the sitl-run skill.
compatibility: PX4 v1.17.0 SITL in WSL Ubuntu-24.04 started by the sitl-run skill; repo at ~/utopia/vibe-coded in WSL.
metadata:
  author: utopia-labs
  version: "1.0"
  verified: "2026-09-14"
---

# PID tuning in SITL

Verified on 2026-09-14 on scenario `atlas_weighed_p26_side30` (real inertia 0.58 / 0.67 / 1.06
kg m², 11.74 kg, nose fans leaned 30 deg sideways, hover pitch 26). Read
`references/session-2026-09-14.md` for the runs and numbers behind every rule below.

## 1. Where the gains come from

`px4_tuning()` in `backend/vectra/export/gazebo.py` sizes every loop from the scenario: rate
crossover w_c = min(4 rad/s, 1 / (3.5 x fan lag)), rate P = w_c / (torque authority / inertia)
per axis, I = 0.6 P, D = 0.05 P, attitude P = w_c / 2.5, position loops below the attitude
bandwidth. `control.px4_params_override` in the scenario wins over the rule and reaches the
SITL airframe, the HITL params and the board push through the same call. The override block is
where tuned values are baked; a value there that was tuned on another geometry is the first
suspect (the 2026-09-13 scenario carried MC_YAWRATE_P 0.08 from a hand tune on forward/aft nose
fans, one seventh of the rule, and the aircraft could not hold heading).

Print rule versus override for a scenario:

```bash
uv run --project backend python -c "import json;from vectra.scenario import Scenario;from vectra.export.gazebo import px4_tuning;sc=Scenario.model_validate(json.load(open('scenarios/<name>.json')));r=px4_tuning(sc.model_copy(update={'control':sc.control.model_copy(update={'px4_params_override':{}})}));c=px4_tuning(sc);[print(k,r.get(k),c.get(k)) for k in c if r.get(k)!=c.get(k)]"
```

## 2. The loop

One parameter group per flight. Live changes persist across `px4ctl.sh reset` but not across a
relaunch, so bake what works into the scenario at the end.

1. Baseline: `hover_run.sh --hover 55 --tag run1` (sitl-run skill), then
   `uv run --project backend python .claude/skills/pid-tuning/scripts/tune_table.py exports/logs/run1_*.ulg`.
2. Read the table (section 4). Decide one change.
3. Fly it: `hover_run.sh --hover 55 --tag run2 NAME VALUE [NAME VALUE ...]`. Chain two runs in one
   background call with `sleep 20` between them when the second does not depend on the first.
4. `tune_table.py exports/logs/run*.ulg` side by side. Keep or revert.
5. Bake: `uv run --project backend python .claude/skills/pid-tuning/scripts/bake_overrides.py scenarios/<name>.json NAME VALUE ... --note "why"`
   writes the kept values into `control.px4_params_override`, appends the note to
   `meta.notes` and re-exports the harness. Then `vectra_gazebo.sh --stop`, `launch_detached.sh`,
   `wait_ready.sh` and one more `hover_run.sh` with no live parameters: the export must
   reproduce the result before anything goes to the board.

`scripts/tune_table.py` prints per axis attitude peak to peak (deg), dominant frequency, RMS rate
error (deg/s), normalised torque peak, plus horizontal drift, altitude standard deviation, motor
mean and max, saturation fraction and the gains the log ran with. Window: 3 s after the vehicle is
above 1 m to 3 s before it is not. Targets in hover: attitude peak to peak under 1 deg, yaw under
3 deg, drift under 0.5 m, rate error RMS under 2 deg/s, no axis with torque peak near 1.

## 3. The trap that is not a gain: allocator residual freezes the integrators

PX4's rate controller stops integrating on an axis in the direction the allocator reports as
unallocated torque, with a threshold of FLT_EPSILON
(`src/modules/mc_rate_control/MulticopterRateControl.cpp:200-215`, `rate_control.cpp:91`). When
the geometry has no exact hover trim (the metrics say `exact False` and the live
`control_allocator_status` shows `torque_setpoint_achieved False` with unallocated torque of
order 1e-3), every axis carries a permanent residual and all three integrators lock in one
direction. The symptom is not a wobble: attitude peak to peak under 1 deg, torque demand tiny,
the attitude setpoint pinned at the tilt limit and the vehicle sliding away at constant
acceleration (270 m in 40 s on run 1), heading drifting with the yaw rate setpoint ignored.

Test before touching any gain: `sitl_probe.sh` mid hover. If `torque_setpoint_achieved` is
`False`, fix the geometry the allocator sees, not the gains. On this aircraft the cause is the
sideways lean of the two nose fans (CA_ROTOR8_AY 0.5, CA_ROTOR9_AY -0.5): declaring them
without lean to the allocator (`CA_ROTOR8_AY 0 CA_ROTOR9_AY 0`) made the trim exact, the
integrators live, and position hold went from 270 m to 3.5 m of drift in the next flight with
the same gains. The real lean stays in the physics (and on the aircraft) as a small disturbance
the integrators absorb. Bake it as `CA_ROTOR8_AY: 0, CA_ROTOR9_AY: 0` in
`control.px4_params_override`; `scenario_to_ca_params` applies CA_ overrides last, so the board
push carries it too. The metrics still build their matrix from the geometry, so they keep saying
`exact False` for this scenario; that is a known gap.

Two more settings that are geometry, not gains, on this aircraft:

- `MPC_YAW_MODE 5` (yaw fixed). The default 0 turns the heading towards the position setpoint;
  a few metres of drift then rotates the heading, the heading change upsets the position loop,
  and heading, roll and pitch all swing together at 0.1 to 0.2 Hz (runs 3 to 7: 300 to 500 deg
  of yaw peak to peak with any yaw gain). With 5, yaw peak to peak went to 6 deg and roll to
  1.2 deg in the next flight (run 8).
- `CA_ROTOR8_CT` and `CA_ROTOR9_CT` scaled by cos(lean) once the lean is hidden from the
  allocator, so the nose fans' modelled lift equals their real vertical component. Without it
  the pitch integrator has to find about 0.05 of pitch authority after every liftoff and the
  aircraft dips 5 deg nose down and slides 2 m sideways in the first seconds (run 9). The
  exporters keep the simulated fan on the fan curve (`Scenario.fan_ct_physical`), so the CT
  override changes what the allocator believes, never what the sim fan does.

## 4. Reading the table

| what you see | meaning | change |
|---|---|---|
| attitude swings with the setpoint swinging too, 0.1 to 0.3 Hz, metres of wander | position loop faster than the attitude loop | lower MPC_XY_VEL_P_ACC and MPC_XY_P first |
| attitude swings, setpoint steady, torque peak near 1 | rate loop ringing at saturation | lower MC_*RATE_P 30 %, tighten MC_*_INT_LIM |
| attitude error steady, torque tiny, setpoint at tilt limit | integrators frozen (section 3) | fix the allocator residual |
| yaw drifts slowly, yaw rate setpoint ignored, yaw torque under 0.05 | yaw gains far below the rule, or MC_YR_INT_LIM below the moment the integrator must carry | MC_YAWRATE_P towards the rule value in two steps, MC_YAW_P with it, MC_YR_INT_LIM 0.3 |
| heading swings hundreds of degrees, roll and pitch swing with it at 0.1 to 0.2 Hz, yaw tracks its own setpoint | the flight task rotates the yaw setpoint towards the position setpoint (`trajectory_setpoint.yaw` moving) | MPC_YAW_MODE 5, not a gain |
| 5 deg pitch dip and metres of sideways slide in the first seconds after liftoff, integrator then settles at a steady value | allocator lift model differs from the fans' real vertical thrust | correct the CA_ROTORn_CT of the fans whose lean is hidden from the allocator |
| yaw limit cycle 0.3 to 0.5 Hz, tens of degrees | attitude yaw P too high for the rate loop | keep MC_YAW_P / MC_YAWRATE_P at the roll and pitch ratio |
| rate error RMS high on one axis, attitude fine | integrator limit too low or I too low | raise MC_*RATE_I towards 0.6 P, limit 0.15 to 0.3 |
| gains in the log 3 to 4 times the airframe's | autotune ran | MC_AT_EN 0, relaunch |
| lifts, then flips within 2 s | gains sized for a heavier placeholder inertia | real mass properties into the scenario first |

## 5. Carrying gains to the aircraft

The SITL fan model is thrust = k x omega² with omega linear in command and one spool constant
both ways; the real EDF and ESC differ, so SITL gains are a starting point, not a result. Push
the scenario (Upload params in the Pixhawk card), fly the ground test with the fans' step
response logged, run `scripts/flight_debug.py` and `hover_report.py` on the board log, and
re-tune on the aircraft with the same one-change rule. Never run PX4 autotune here.
