# PLAN.md: EDF Tilt Geometry Simulator ("tiltlab")

Generated 2026-09-09 16:13 (America/Los_Angeles). Drop this file in the repo root as `PLAN.md`.
Claude Code reads it section by section; it never needs to be loaded whole.

---

## 0. Purpose

A browser app, shared by a team, that:

1. Simulates the 10-EDF aircraft (static analysis and 6-DOF dynamics), with the real PX4 controller in the loop (SITL over TCP, or the Pixhawk 6X Pro over USB in HITL) and a built-in PX4 replica when no PX4 is connected.
2. Ingests the STEP model, auto-detects the 10 fans, takes per-body masses, computes the inertia tensor, and checks itself against the CAD-reported mass and CG.
3. Lets the user tilt every fan in any direction (0 to 90 degrees, orientation only, positions fixed, count fixed at 10) and see, live, what each geometry costs and buys on all six axes, in both the stock-PX4 (under-actuated) concept and the fully-actuated concept, with a transition control between them.
4. Exports everything as tick-box options: PX4 `.params`, JSON scenario, CSV metrics, plots, Markdown/PDF report, Gazebo SDF, mechanical angle sheet.

Every generated file is named `YYYYMMDD_HHMM_<name>.<ext>` (local time).

---

## 1. Facts this plan relies on (verified in conversation and logs)

| Fact | Source |
|---|---|
| Airframe: 10 XFly 80 mm 12-blade EDFs. Rotors 0 to 7 on a 30 degree dihedral surface behind the CG, thrust axis normal to the surface (up and inward). Rotors 8, 9 vertical on the centreline ahead of the CG. Positions in metres (FRD, relative to CG) are the CA_ROTORn_PX/PY/PZ values in the current parameter file. | logs 27, 36, 40; user statements |
| All fans spin CW. | user |
| Flight controller: Pixhawk 6X (FMUv6X target, `PX4_FMU_V6X`, V6X006 in logs). User will use the Pixhawk 6X Pro with the latest PX4. Both are the `px4_fmu-v6x` build target. | log info, PX4 docs |
| PX4 1.17 control allocation: 6-axis effectiveness matrix (roll, pitch, yaw torque, Fx, Fy, Fz), pseudo-inverse, per-axis normalisation (`ControlAllocationPseudoInverse::updateControlAllocationMatrixScale`), desaturation per `CA_METHOD` (0 = pseudo-inverse with clipping, 1 = sequential desaturation, 2 = automatic). Stock multicopter controllers command thrust along body Z only (`thrust_body = (0, 0, -T)`), so Fx and Fy setpoints are always zero unless a controller that publishes 3D thrust is used. | PX4 source v1.17.0, logs (Fx row caused 94 percent unallocated thrust in log 36) |
| PX4 has an "Omnicopter" reference build: generic multicopter airframe, tilted rotors in CA_ROTOR geometry, `CA_METHOD = 0`, `FD_FAIL_P = FD_FAIL_R = 0`. It uses reversible motors (`CA_R_REV = 255`), which EDFs cannot do. Independent research groups state PX4 does not natively translate at level attitude; they patch `mc_pos_control` to publish the 3D thrust vector instead of tilting. | docs.px4.io omnicopter page; arXiv 2511.15909; castacks/PX4-fully-actuated |
| PX4 HITL: community supported, "may or may not work with current versions". Compatible airframe `SYS_AUTOSTART = 1001` (HIL Quadcopter X). Requires `pwm_out_sim` in firmware (check with `pwm_out_sim status` in the MAVLink console; `boards/px4/fmu-v6x/default.px4board` is the reference config). Simulator connects over USB serial (921600 baud, jMAVSim uses 250 Hz) and bridges MAVLink to QGC over UDP. | docs.px4.io HITL page |
| PX4 SITL accepts any external simulator over TCP port 4560 using the same HIL_* MAVLink messages (lockstep). | PX4 SITL architecture (jMAVSim, Gazebo Classic use it) |
| XFly 80 mm 12-blade, 6S: 3280-KV2200 variant about 3400 g max static thrust at about 100 A, 320 g; 3665-KV2300 PRO variant about 3650 g at about 120 A, 340 g. Only max-thrust points are published. A full thrust and power curve must be entered from a thrust stand or estimated. | manufacturer listings |
| Rig: locks roll and position; pitch and yaw are free. | user correction |
| Logs 27, 36, 40 and parameter files are available as ground truth for the allocator and controller replicas (they contain `control_allocator_status`, `actuator_motors`, torque/thrust setpoints, rate setpoints, and the parameters in force). | uploaded files |

## 2. Corrections to earlier assumptions

- Nominal thrust `CT = 6.5 N` in the current parameter file is a placeholder. Real max thrust per fan is about 33 to 36 N. The app must never hard-code CT; it derives CT from the fan curve.
- The rig constrains roll only, not pitch.
- The KM-based yaw model in the current params is a workaround for stock PX4 ignoring Fy. In the fully-actuated concept the faithful geometry (AY tilt) is the right model. The app must be able to produce both.

---

## 3. Decisions (fixed unless the user changes them)

| Topic | Decision | Why |
|---|---|---|
| Frontend | Vite + React 18 + TypeScript, `@react-three/fiber` + `drei` for 3D, `zustand` for state, `uplot` for time series, `plotly.js-dist-min` only for the polytope plots. Tailwind for layout. | Small, fast, well-known to Claude Code, minimal boilerplate. |
| Backend | Python 3.11, FastAPI + uvicorn, numpy, scipy, `cadquery-ocp` (OpenCascade), `pymavlink`, `pyulog`, `pyserial`. | STEP B-rep needs OpenCascade; the physics, allocator replica and MAVLink bridges live in one process so there is exactly one physics implementation. |
| Where physics runs | Backend. Browser receives state over WebSocket at 60 Hz. | HITL and SITL need the sim next to the serial/TCP link; avoids a second physics engine in JS. |
| Frames and units | PX4 body frame FRD (X forward, Y right, Z down), NED world. SI units everywhere internally (m, kg, N, N m, W, rad). Degrees only in the UI. | Matches PX4, removes conversion bugs. |
| Fan orientation parameterisation | Per fan: tilt θ in [0, 90] deg and azimuth φ in [0, 360) deg. Thrust direction `a = (sinθ cosφ, sinθ sinφ, -cosθ)` in FRD. φ = 0 tilts the thrust forward, φ = 90 tilts it to the right. Optional mirror lock between left/right pairs (default on, user can switch off). | Unambiguous, maps directly to CA_ROTORn_AX/AY/AZ. |
| Allocator | Verbatim port of PX4's `ActuatorEffectivenessRotors` matrix construction, `ControlAllocationPseudoInverse` (including `updateControlAllocationMatrixScale`) and `ControlAllocationSequentialDesaturation`, selectable by `CA_METHOD`. | Results must transfer to the real vehicle. |
| Controller replica | PX4 `mc_rate_control` (K, P, I, D, FF, integrator limit, yaw torque cutoff) and `mc_att_control` (P, yaw weight) with PX4 parameter names, plus two attitude strategies for the fully-actuated concept (zero-tilt, full-tilt) and a blend slider. | Gains transfer; the strategy switch is the requested "transition". |
| PX4 in the loop | SITL: our sim is the external simulator on TCP 4560. HITL: same code over serial. Both use HIL_SENSOR, HIL_GPS, HIL_STATE_QUATERNION out, HIL_ACTUATOR_CONTROLS in, and bridge MAVLink to QGC on UDP 14550. | One bridge, two transports. |
| Fan detection in STEP | Cylindrical B-rep faces with radius 38 to 45 mm and axial extent over 40 mm, clustered by axis. Axis sign confirmed by the user (default: thrust points to body -Z). | 80 mm rotor, 84 mm duct: robust signature. |
| Mass properties | Per-solid volume and inertia from tessellated meshes (divergence theorem), user-assigned mass per solid (density or absolute), totals checked against CAD mass and CG. | User's CAD gives mass and CG only. |
| Deployment | `docker compose up` for the shared analysis server. Native Python run (`uv run`) on the machine that has the Pixhawk, because USB passthrough into Docker is unreliable on Windows and macOS. | Team access plus HITL. |
| Fidelity | Rigid body 6-DOF, RK4 at 1000 Hz, first-order fan lag, thrust and power from the fan curve, quadratic body drag, gravity, ground plane, rig constraints. No aerodynamic interaction between ducts (recommended and accepted). Reaction torque toggle with KM default 0 (EDF stators recover most swirl). | Accepted in Q11. |
| Additional libraries | Backend: `websockets` (WebSocket transport for the 60 Hz state stream), `pydantic>=2` (Scenario schema validation), `python-multipart` (STEP and params upload), `trimesh` (mesh inertia and STL handling), `fast-simplification` (decimate tessellated meshes for the browser), `pygltflib` (glTF export of the aircraft mesh), `matplotlib` (static report plots), `jinja2` (report templating), `markdown` (Markdown to HTML for the report), `httpx` (FastAPI TestClient and outbound requests), optional extra `weasyprint` (PDF report, may be missing on Windows), `hatchling` (build backend); dev: `pytest`, `pytest-asyncio`, `ruff`, `mypy`. Frontend: `three` (explicit peer of fiber and drei), `@tailwindcss/vite` (Tailwind v4 Vite plugin), `vitest`, `@testing-library/react`, `@testing-library/jest-dom`, `jsdom` (component tests), `@vitejs/plugin-react` (React fast refresh), `@types/react`, `@types/react-dom`, `@types/three` (type declarations). | Declared once in M1 so later modules never edit the dependency lists in `backend/pyproject.toml` or `frontend/package.json`. |

---

## 4. Architecture

```
browser (React/TS)  <-- WebSocket 60 Hz state, REST for CAD/params -->  backend (FastAPI)
                                                                         |-- core/geometry.py     effectiveness matrix (PX4 port)
                                                                         |-- core/allocation.py   pinv + scaling + desaturation (PX4 port)
                                                                         |-- core/controller.py   PX4 rate/att replica + strategies
                                                                         |-- core/fan.py          thrust/power curves, lag
                                                                         |-- core/rigid_body.py   6-DOF, rig constraints, sensors
                                                                         |-- core/metrics.py      hover, authority LP, power, coupling
                                                                         |-- cad/step_import.py   OCP: solids, cylinders, mesh
                                                                         |-- cad/mass_props.py    inertia, CG check
                                                                         |-- px4/mavlink_hil.py   SITL (tcp) / HITL (serial) bridge
                                                                         |-- px4/params.py        .params read/write, MAVLink param sync
                                                                         |-- export/*.py          params, csv, report, sdf, angle sheet
```

Single source of truth: a `Scenario` JSON (section 5). Everything (UI sliders, exports, sim, PX4 params) reads and writes that object.

---

## 5. Data model (JSON, stored in `scenarios/*.json`)

```jsonc
{
  "meta": {"name": "baseline_dihedral30", "created": "2026-09-09T16:13:00-07:00", "px4_version": "1.17.0"},
  "frame": {"cad_forward_axis": "+X", "cad_up_axis": "+Z", "cad_units": "mm"},   // user-confirmed, used to map CAD -> FRD
  "mass": {"total_kg": 0.0, "cg_frd_m": [0,0,0], "inertia_frd_kgm2": [[0,0,0],[0,0,0],[0,0,0]],
           "cad_reported": {"mass_kg": 0.0, "cg_m": [0,0,0]}, "bodies": [{"name": "", "volume_m3": 0, "mass_kg": 0, "source": "user|density|cad"}]},
  "fans": [ {"id": 0, "output": "MAIN1", "pos_frd_m": [-0.30,-0.37,-0.14], "tilt_deg": 30, "azimuth_deg": 90,
             "spin": "CW", "mirror_of": 1, "curve_ref": "xfly80_3280"} ],   // 10 entries, ids 0..9
  "fan_curves": {"xfly80_3280": {"cells": 6, "points": [{"cmd": 0.0, "thrust_N": 0, "power_W": 0}, {"cmd": 1.0, "thrust_N": 33.3, "power_W": 2450}],
                                  "lag_s": 0.15, "max_continuous_A": 100, "notes": "manufacturer max only; replace with thrust stand data"}},
  "control": {"concept": "stock|fully_actuated", "blend": 0.0, "ca_method": 0, "px4_params_override": {"MC_ROLLRATE_P": 0.15}},
  "rig": {"enabled": true, "lock_position": true, "lock_roll": true, "lock_pitch": false, "lock_yaw": false, "attitude_offset_deg": [0,6,0]},
  "environment": {"air_density": 1.225, "wind_ned_mps": [0,0,0], "gravity": 9.80665},
  "outputs": {"params": true, "csv": true, "report": true, "plots": true, "sdf": false, "angle_sheet": true}
}
```

Derived (never stored): AX/AY/AZ from tilt and azimuth; CT from `fan_curves[...].points` (thrust at cmd = 1.0); KM from the reaction-torque toggle.

Implemented additions (M2, 2026-09-09): `fans[].km` holds the KM magnitude per fan (sign from spin) so the flown KM yaw model can be represented; `control.reaction_torque` is the toggle (default false, meaning KM = 0 everywhere); `control.px4_params_override` may carry `CA_ROTORn_CT` overrides so the flown scenarios reproduce the logged CT (6.5 / 5.6 N) instead of the fan-curve value. The model lives in `backend/tiltlab/scenario.py`.

---

## 6. Modules, in build order, each with its acceptance test

Each module is one Claude Code session. Do not start the next before the test passes.

### M1  Repo scaffold (30 min)
- Monorepo: `backend/` (Python, `pyproject.toml`, `uv`), `frontend/` (Vite), `scenarios/`, `tests/fixtures/` (copy `log_36_2026-9-9-14-11-36.ulg`, `log_40_2026-9-9-15-09-54.ulg`, `14150909.params`, the v3 and v4 params files).
- `CLAUDE.md` (section 8), `Makefile` targets: `dev`, `test`, `lint`, `docker`.
- Accept: `make test` runs an empty pytest and vitest; `make dev` serves a blank page from the backend.

### M2  Geometry and allocator replica (core, highest priority)
- `core/geometry.py`: `effectiveness_matrix(fans, cg, ct, km) -> 6x10` exactly as PX4 builds it (thrust rows 3 to 5 as `ct * axis`, moment rows as `ct * cross(pos, axis) + reaction term`). Port the sign convention from PX4 source (`src/lib/control_allocation/actuator_effectiveness/`, tag `v1.17.0`); do not rely on memory.
- `core/allocation.py`: pseudo-inverse with PX4 normalisation and the three `CA_METHOD` behaviours; actuator limits [0, 1]; returns `actuator_sp`, `unallocated_torque`, `unallocated_thrust`, `saturation` per actuator.
- `core/params_px4.py`: read/write PX4 `.params` (tab-separated, CRLF), map Scenario <-> `CA_ROTORn_*`, `CA_ROTOR_COUNT`, `CA_METHOD`.
- Accept (golden test, must pass within tolerance): load parameters from log 36 and log 40 with `pyulog`; feed the logged `vehicle_torque_setpoint` and `vehicle_thrust_setpoint` through the replica; compare against logged `actuator_motors` and `control_allocator_status.unallocated_*`. Target: RMS error < 0.02 on motor commands, unallocated thrust sign and magnitude within 5 percent (log 36 at 33 s: unallocated thrust z = -0.936, rotors 0 to 7 saturated low; log 40: 78 percent of yaw torque unallocated over yaw commands).

### M3  Fan model
- `core/fan.py`: piecewise-linear thrust(cmd) and power(cmd) from `fan_curves`, inverse `cmd(thrust)`, first-order lag, optional battery voltage sag (cells, internal resistance, capacity), current estimate. Default curve for XFly 80 mm from the published max point using `thrust ∝ power^(2/3)`, flagged `estimated: true` in the UI until replaced.
- Accept: unit tests for monotonicity, inverse round-trip, and that CT reported to PX4 equals thrust at cmd = 1.0.

### M4  Static analysis and metrics
- `core/metrics.py`, for a Scenario and a concept (stock: controlled axes roll, pitch, yaw, Fz with Fx = Fy = 0 as PX4 does; fully actuated: all six):
  - Hover solution `u_hover` (weight from mass), per-fan thrust, total electrical power, hover headroom `1 - max(u)`.
  - Per axis, ± authority from hover: maximise |axis| subject to `B u = target`, `0 <= u <= 1` (scipy `linprog`). Report N m or N, and per-watt values.
  - Marginal power per unit command per axis (finite difference).
  - Coupling matrix: apply 20 percent of authority per axis through the desaturating allocator, measure off-axis leakage.
  - Conditioning: singular values of the normalised B restricted to controlled axes; rank; null-space dimension.
  - Composite score = Σ weights × normalised metrics; weights editable; tick boxes select which metrics display and export.
- Accept: with the current parameter geometry the metrics reproduce the known facts: stock concept with all-vertical KM model has yaw authority about one third of roll; faithful AY model under stock concept needs about ±4.8 units on inner fans per 0.5 N m yaw (i.e. yaw is unusable). Print a table.

### M5  Frontend: explorer
- Layout: left panel fan table (10 rows: tilt, azimuth, mirror lock, output, spin), centre 3D view (mesh or stick model, thrust vectors scaled by hover thrust, CG marker, FRD axes, rig gimbal ghost), right panel metrics with tick boxes, top bar concept switch (stock / fully actuated / blend slider), snapshot A/B compare, presets (current dihedral 30, all vertical, omni-style).
- Live: every slider change posts the Scenario, backend returns metrics in under 50 ms.
- Accept: changing a fan tilt updates thrust vector, metrics and PX4 param preview without page reload; A/B diff table works; keyboard nudge ±1 degree.

### M6  CAD pipeline
- `cad/step_import.py` (OCP): read STEP, list solids with names, tessellate (linear deflection 0.5 mm), detect fan cylinders (radius 38 to 45 mm, axial extent > 40 mm), cluster to fan candidates, expose centre and axis line; confirm count = 10 or let the user pick faces in the 3D view.
- `cad/mass_props.py`: volume, centroid, inertia per solid from mesh; mass assignment UI (per solid: absolute mass, or density with material presets; "cad" source if STEP carries density); totals; compare with CAD-reported mass and CG, show error in grams and millimetres; fail the import if error > 2 percent mass or > 5 mm CG until the user overrides.
- Frame mapping UI: user picks CAD forward and up axes (defaults suggested from the two fan rows: the two centreline fans are forward). Positions converted to FRD relative to computed CG.
- Fan axis sign: default thrust toward body -Z; per-fan flip; sanity check that summed thrust direction points up.
- Mesh exported as glTF for the frontend viewer, decimated to under 200k triangles.
- Accept: import the user's STEP; 10 fans detected; recomputed CG within tolerance of CAD CG; CA_ROTOR positions within 5 mm of the current parameter file values (they were measured from this CAD).

### M7  Dynamic simulation
- `core/rigid_body.py`: state (pos NED, vel, quaternion, body rates), RK4 1000 Hz, forces and moments from fans through `fan.py` lag, gravity, quadratic drag (Cd·A per axis, user-set), ground plane (spring-damper, friction), rig constraints (position lock; per-axis rotation lock implemented as constraint that zeroes the locked body-rate component and holds the locked angle each step, with reaction torque reported so integrator wind-up is visible), attitude offset to reproduce a rig that is not level.
- `core/sensors.py`: IMU (accel with gravity, gyro), magnetometer (NED field rotated to body, default field for user location), barometer (ISA), GPS (lat/lon/alt from NED at a home point), configurable noise, 250 Hz output; also ground-truth state.
- `core/controller.py`: PX4 replica (rate PID, attitude P, yaw weight, `MC_YAW_TQ_CUTOFF`, manual throttle curve with `MPC_THR_HOVER`, `MPC_MANTHR_MIN`, `MPC_THR_CURVE`, spool-up ramp, landed-throttle-floor behaviour observed in log 27) driving `allocation.py`. Attitude strategies: `full_tilt` (stock: translate by tilting, thrust along body Z), `zero_tilt` (fully actuated: level attitude, 3D body thrust), and `blend` in [0, 1] mixing the two attitude setpoints. Inputs: virtual sticks from the UI or a gamepad via the browser Gamepad API.
- Accept (replay test): drive the replica with the sticks recorded in log 40 (`manual_control_setpoint`), rig mode on, attitude offset (roll -2, pitch +6 deg) as in the log; the replica's torque setpoints and yaw unallocated fraction must track the log within 15 percent RMS. Second test: log 36 geometry must reproduce "rotors 0 to 7 pinned at zero".

### M8  PX4 in the loop
- `px4/mavlink_hil.py`: one class, two transports. Outbound at 250 Hz: HIL_SENSOR (time_usec, accel, gyro, mag, abs_pressure, pressure_alt, temperature, fields_updated), HIL_GPS at 10 Hz, HIL_STATE_QUATERNION at 50 Hz, HEARTBEAT 1 Hz. Inbound: HIL_ACTUATOR_CONTROLS (16 floats, motors 0..1) applied as fan commands. QGC bridge: forward all other MAVLink both ways to UDP 14550.
  - SITL: connect to `tcp:127.0.0.1:4560`; lockstep: advance the sim one step per received HIL_ACTUATOR_CONTROLS and stamp `time_usec` from sim time. Start PX4 with `make px4_sitl none_iris` (verify exact target name in the checked-out PX4 tree), then push the Scenario's CA_ROTOR params over MAVLink `PARAM_SET` and reboot.
  - HITL: serial port at 921600, real time (no lockstep). Pre-flight checklist enforced in UI: fans unpowered, `SYS_AUTOSTART = 1001` applied and rebooted, `pwm_out_sim status` returns OK (send via MAVLink SERIAL_CONTROL and parse), then push CA_ROTOR_COUNT = 10 and geometry, `COM_RC_IN_MODE` as required, arm via QGC or joystick.
- `px4/params.py`: read all params from the vehicle (PARAM_REQUEST_LIST), diff against Scenario, push only differences, verify readback, write timestamped `.params` backup before any push.
- Accept: (1) SITL with the stock `x500` geometry hovers in Position mode from a virtual takeoff, proving the bridge before touching the custom geometry. (2) SITL with the current 10-fan geometry reproduces the log 36 behaviour when AX = +1 is set (rear fans idle, front pair only). (3) HITL with airframe 1001 on the Pixhawk: PX4 shows HIL enabled, sensors healthy, arms, HIL_ACTUATOR_CONTROLS arrive at ≥ 200 Hz.

### M9  Exports (tick boxes, all timestamped)
- PX4 `.params`: only CA_* geometry plus the concept-specific set (stock: unchanged `CA_METHOD`; fully actuated: `CA_METHOD = 0`, `FD_FAIL_P = FD_FAIL_R = 0` per PX4 omnicopter guidance), merged onto the user's latest full parameter backup so nothing else changes. Header comment lists tilt/azimuth per fan and the fan curve used.
- CSV: all metrics for A, B and every saved snapshot.
- Plots: PNG of the attainable torque and force polytopes (2D projections), power vs command per fan, step responses.
- Report: Markdown, optional PDF (weasyprint): geometry table, mass properties with CAD check, metrics, plots, assumptions (fan curve estimated or measured), PX4 caveats (section 9).
- Gazebo SDF: model with 10 rotor links and `gazebo_motor_model` plugins, thrust constant from the fan curve, for teams that want Gazebo Classic; generated from the Scenario, not hand-edited.
- Mechanical angle sheet: per fan, tilt and azimuth in body frame, unit vector, and the equivalent two rotation angles about the mount's own axes in the CAD frame, so the machinist gets numbers in the frame they model in.

### M10  Team deployment and docs
- `docker-compose.yml`: backend (with OCP), frontend static build served by the backend, volume for `scenarios/`. `README.md`: 10-line quick start, HITL native-run instructions, safety notes.
- Accept: fresh clone, `docker compose up`, open `http://host:8000`, load `scenarios/baseline_dihedral30.json`, metrics render.

---

## 7. Validation gates ("make no mistakes" translated into checks)

| Gate | What must be true before proceeding |
|---|---|
| G1 Allocator | M2 golden tests pass on logs 36 and 40. If they do not, the port is wrong; fix the port, never tune tolerances. |
| G2 Mass | M6 recomputed mass and CG match the CAD report within 2 percent / 5 mm. |
| G3 Replica | M7 replay tests pass; the replica must reproduce the yaw starvation of log 40 and the idle rear fans of log 36. |
| G4 Bridge | M8 test 1 (stock x500 in SITL) passes before any custom geometry is loaded into PX4. |
| G5 HITL | Fans physically disconnected from power for every HITL session; UI blocks the HITL connect button until the checklist is ticked. |
| G6 Export | Round trip: export `.params`, re-import, Scenario identical; load into PX4 SITL, read back, identical. |

---

## 8. CLAUDE.md (paste into the repo root; this is the token budget)

```markdown
# tiltlab conventions
- Read PLAN.md only by section (`grep -n "^### M" PLAN.md` to find offsets). Never cat the whole file.
- One module per session. Finish with `make test` green and one commit: "M<n>: <one line>".
- Python: numpy-vectorised, type hints, docstrings only where units or frames are involved. Every function that takes a vector states its frame (FRD/NED) and unit in the docstring.
- Frames: FRD body, NED world, SI units. Degrees only in UI components.
- Never hard-code CT, mass, inertia or fan positions in code. They come from the Scenario.
- PX4 behaviour: port from the pinned source tree in `third_party/PX4-Autopilot` (git tag v1.17.0, sparse checkout of src/lib/control_allocation, src/modules/mc_rate_control, src/modules/mc_att_control, src/modules/simulation/pwm_out_sim, msg/). Cite the file and line in a comment. Do not paraphrase from memory.
- Tests are golden-data tests against tests/fixtures/*.ulg. Do not loosen tolerances to pass.
- Frontend: components under 150 lines, state in zustand store, no prop drilling deeper than two levels, no new dependencies without a line in PLAN.md section 3.
- Do not print large arrays, logs or STEP contents into the conversation. Write them to files and summarise in one line.
- Do not explain what you are about to do. Do it, run the test, report pass/fail in one line.
- Commands: `make dev` (backend :8000 + vite :5173), `make test`, `make lint`, `make docker`.
```

Token economy rules for the human driving Claude Code:
- Start each session with: "Implement M<n> from PLAN.md. Read only that section and CLAUDE.md." Nothing else.
- Use `/compact` after every green test run. Never carry more than one module of context.
- Give feedback as failing test cases, not prose.
- When a PX4 fact is uncertain, the instruction is "open the pinned source file and quote the lines", not "recall".

---

## 9. Known unknowns and how the app handles them

| Unknown | Handling |
|---|---|
| Whether PX4 HITL works on 1.17 / 1.18 with `px4_fmu-v6x` and `pwm_out_sim`. | Checked at M8 acceptance 3. Fallback: SITL with the identical bridge; HITL becomes optional. |
| Whether stock PX4 can command body-lateral thrust in any pilot-facing mode. Evidence says no (thrust along body Z only; research groups patch `mc_pos_control`). PX4 main has a `TrajectorySetpoint6dof` uORB message; whether any stock mode consumes it is unverified. | The app shows two capability badges on every metric: "usable with stock PX4" and "needs fully-actuated controller". Fully-actuated results are only achievable in flight with a patched PX4 (e.g. castacks fork approach) or an offboard controller; the report says so on page one. |
| Fan thrust and power curve. | Estimated from the manufacturer max point until thrust-stand data is entered; every metric carries an "estimated" flag until then. |
| Reaction torque of the EDFs. | Toggle, default 0; if a rig yaw test with all fans equal shows drift, fit KM from it. |
| CAD frame orientation and units. | User confirms once per import; stored in Scenario. |
| Mass of each body. | User assigns; CAD CG check catches mistakes. |
| Exact SITL make target for an external simulator in the checked-out PX4 version. | Verify in `Tools/simulation/` and `platforms/posix/cmake/sitl_target.cmake` at M8; write the verified command into README. |

---

## 10. What the physics must get right (checklist for M2, M4, M7)

- Thrust force `F_i = T_i · a_i` at position `r_i` (relative to CG); moment `M_i = r_i × F_i + (reaction torque) · a_i` with the PX4 sign convention for KM.
- Allocation is linear in normalised thrust `u ∈ [0, 1]`; the non-linear command-to-thrust map lives in the fan model and in PX4's `THR_MDL_FAC`, not in the effectiveness matrix.
- Stock concept: Fx and Fy setpoints are zero and are still rows of the matrix; that is what starved the rear fans in log 36 and what makes yaw expensive with faithful AY geometry. The app must reproduce this, not hide it.
- Sequential desaturation drops yaw first at saturation; with low collective on the rig this produced 78 percent unallocated yaw in log 40. The static metrics must therefore be reported at a stated collective (default hover), and the UI must show a "collective" slider so the rig condition can be reproduced.
- Rig constraints produce reaction torques; the integrators of locked axes wind up exactly as they did in logs 27 and 40 (pitch integrator at -0.3). Show integrator state in the UI.
- Gyroscopic torque of fan rotors is small but non-zero for 12-blade nylon rotors at 30k+ rpm; include as an optional term with rotor inertia from the fan model (default off until inertia is known).

---

## 11. First three Claude Code prompts (copy verbatim)

1. `Implement M1 from PLAN.md. Read only section 6/M1 and section 8. Create CLAUDE.md from section 8. Pin PX4 v1.17.0 as a sparse git checkout into third_party/ with only the paths listed in CLAUDE.md. Finish with make test green.`
2. `Implement M2 from PLAN.md. Read only section 6/M2, section 5, section 10, and the pinned PX4 files under third_party/PX4-Autopilot/src/lib/control_allocation. Port the effectiveness matrix, pseudo-inverse scaling and desaturation verbatim with file:line comments. Write the golden test against tests/fixtures/log_36*.ulg and log_40*.ulg as specified. Do not tune tolerances.`
3. `Implement M3 and M4 from PLAN.md. Read only sections 6/M3, 6/M4 and 5. Acceptance is the printed metrics table in M4.`

Then M5 through M10 in order, one per session.
