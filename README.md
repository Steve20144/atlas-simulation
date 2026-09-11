# tiltlab

Browser app for the 10-fan EDF aircraft. Tilt every fan (0 to 90 degrees, any azimuth), see what the geometry costs and buys on all six axes through a line-by-line replica of the PX4 v1.17.0 control allocator, and export the result as PX4 parameters, CSV and scenario JSON.

Frames are FRD body (X forward, Y right, Z down) and NED world. SI units internally, degrees only in the UI.

## What is in this version (lean v1)

| Piece | Where | State |
|---|---|---|
| Scenario model (the single source of truth) | `backend/tiltlab/scenario.py`, `scenarios/*.json` | done |
| PX4 effectiveness matrix and allocator port | `backend/tiltlab/core/geometry.py`, `allocation.py` | done, golden-tested against flight logs 36 and 40 |
| PX4 `.params` read/write and Scenario mapping | `backend/tiltlab/core/params_px4.py` | done |
| Fan model (curves, inverse, lag, battery) | `backend/tiltlab/core/fan.py` | done, default curve is an estimate |
| Static metrics (hover, authority, power, coupling, conditioning, score) | `backend/tiltlab/core/metrics.py` | done |
| REST API | `backend/tiltlab/api/app.py` | done |
| Explorer UI (fan table, 3D view, metrics, params preview, exports) | `frontend/src` | done |
| Exports (`.params`, CSV, scenario JSON) | `backend/tiltlab/export` | done |

Deferred to a later version: CAD/STEP import and mass properties (M6), 6-DOF dynamics and the PX4 controller replica (M7), PX4 SITL/HITL in the loop (M8), PDF report, Gazebo SDF, plots, mechanical angle sheet, A/B snapshot compare, gamepad input. The full plan is `PLAN.md`; read it by section (`grep -n "^### M" PLAN.md`).

## Install

Requirements: `uv` (manages Python 3.11 for you), Node 20 or newer, GNU make, git. Docker only for the shared server.

Windows 11 (Git Bash):

```bash
source scripts/env.sh          # puts uv, make (~/.local/bin) and node/npm on PATH for this shell
uv sync --project backend      # creates backend/.venv with Python 3.11; ignore the "minor version link" warning
npm --prefix frontend install
scripts/fetch_px4.sh           # optional: pinned PX4 v1.17.0 sparse checkout into third_party/ (for reading the ported source)
```

In PowerShell use `. .\scripts\env.ps1` instead of `source scripts/env.sh`. Never run the system Python; always go through `uv run --project backend ...`.

Linux and macOS: install `uv`, `make` and Node, then run the same commands. `scripts/env.sh` is harmless there.

## Run

```bash
make dev
```

Backend on http://127.0.0.1:8000 (uvicorn with reload) and the Vite dev server on http://localhost:5173 (proxies `/api` to the backend). Open the Vite URL while developing. For a single-process run, build the frontend once (`npm --prefix frontend run build`) and open http://127.0.0.1:8000, which serves `frontend/dist`.

```bash
make test    # pytest (73 tests, including the golden allocator tests) and vitest
make lint    # ruff and tsc --noEmit
make docker  # docker compose build (not run on the development machine)
```

## Using the explorer

1. **Load a scenario** from the picker in the top bar. Three ship with the repo:
   - `baseline_dihedral30`: the faithful geometry, rotors 0 to 7 tilted 30 degrees up and inward on the dihedral, rotors 8 and 9 vertical.
   - `flown_log40_vertical_km`: all axes vertical with the KM yaw model, as flown in log 40.
   - `flown_log36_ax1`: 45 degree forward tilt (AX = 1) as flown in log 36. It has no feasible stock hover; the app shows that rather than hiding it.
2. **Edit fans** in the left table: tilt (0 to 90) and azimuth (0 to 360) per fan, number inputs or arrow keys for 1 degree steps. Azimuth 0 tilts the thrust forward, 90 to the right. With the mirror lock on (default), editing a fan also edits its left/right partner with mirrored azimuth.
3. **Presets**: Dihedral 30 reloads the baseline file, All vertical sets every tilt to 0, Omni-style sets tilt 45 with azimuths alternating 0/90/180/270.
4. **Concept switch**: Stock evaluates roll, pitch, yaw and Fz with Fx = Fy = 0 exactly as PX4 does (the Fx and Fy rows stay in the matrix, which is what starves tilted fans). Fully actuated evaluates all six axes. Badges on every metric say which one applies: `stock` is reachable with stock PX4, `fully act.` needs a fully-actuated controller.
5. **Collective slider**: metrics are computed at a stated collective (fraction of the thrust with all fans at full command). In hover mode (the default, `hover` button lit) the backend solves for the collective that carries the scenario mass. Drag the slider to reproduce a rig condition such as the low collective of log 40.
6. **Read the metrics** on the right, tick boxes hide groups. Hover: per-fan command `u` (also shown in the fan table and as vector length in the 3D view), power, headroom. Authority: largest attainable plus and minus step per axis with the other axes held at zero, and per watt. Coupling: off-axis leakage when 20 percent of each authority is pushed through the PX4 desaturating allocator. Conditioning: singular values, rank, null space. Composite: weighted score.
7. **PX4 params preview** shows the `CA_ROTORn_*` lines the current geometry produces. **Export** writes a timestamped `.params` (only CA_* geometry, plus `CA_METHOD 0` and `FD_FAIL_P/R 0` for the fully-actuated concept, merged onto your latest full parameter backup so nothing else changes) and a metrics CSV into `exports/`.
8. **Save** writes the edited scenario back to `scenarios/<name>.json`.

The 3D view is a stick model: cylinders at the fan positions oriented along the thrust axis, thrust vectors scaled by hover `u`, CG marker, FRD axes. Orbit with the mouse.

### The foil model (how the Atlas actually vectors thrust)

The eight wing XFly motors are mounted horizontally (motor axis along the fuselage, exhaust aft) and blow into a foil behind them that turns the jet downward. The force on the aircraft therefore acts at the foil, not at the motor, and points where the deflection sends the jet: a deflection of 0 degrees pushes straight forward, 90 degrees is pure lift, 180 degrees pushes straight back. As built the foils sit at 45 degrees, so the thrust vector is 45 degrees up and forward (the PX4 axis 0.707, 0, -0.707 flown in log 36).

In the scenario this is the `foils` section: `left` and `right`, each with its motor ids, `deflection_deg`, a per-motor override for a segmented foil, the `pressure_points_frd_m` where the force acts (the centroid of the foil channel behind each motor, measured on the CAD meshes, about 90 mm aft of the motor centre) and `loss_at_90deg`, the fraction of motor thrust lost when the jet is turned 90 degrees (not measured, 0 by default). Everything downstream (effectiveness matrix, metrics, PX4 `CA_ROTOR*` export) uses the deflected direction, the pressure point and the reduced thrust. The two centreline fans are plain vertical fans.

In the UI the left column becomes **Geometry**: one slider per foil (linked by default), the resulting thrust direction in words, optional per-motor angles for a segmented foil, the turning loss, and the centreline fans. The 3D view draws the motors as grey ducts along the fuselage, a dotted jet to the pressure point, and the force arrow from there.

### The Coanda effect: how far a foil can turn the jet

The jet follows the curved foil by the Coanda effect and detaches once the wrap exceeds a separation angle that depends on jet thickness over surface radius: `theta_sep = 245 deg * exp(-1.64 * h / R)`. For the PHASE_0.1 foils (radius about 0.25 m from the CAD channel walls, 80 mm jet) that is about 145 degrees. Thrust retained while attached is 1 minus 0.10 per 90 degrees of turning; a separated jet leaves at the separation angle with a further 30 percent loss. The constants live in `foils[].coanda` and in the **Coanda surface** block of the Geometry panel; they are engineering estimates until the rig measures the deflected jet angle and thrust at a few wrap settings. The sweep marks any geometry that asks for more turning than the surface can hold as "jet separates", the foil cards say whether each jet is attached, and the effective (not requested) turning drives the thrust direction everywhere, including the PX4 export. `docs/coanda_evaluation.md` has the evaluation of thrust and authority under this model.

### From a sweep result to the Fusion model: the foil design sheet

Open **Foil design sheet (for the CAD)** at the bottom of the Geometry panel. For every wing motor it lists the chosen deflection, the change from the as-built 45 degree foil (the rotation to apply to that foil segment about the lateral axis, positive turns the exit further down and forward), the exhaust direction as a unit vector in the Fusion frame, and the Fusion coordinates (mm) of the motor centre and of the pressure point where the turned jet acts. Model each foil segment so its exit plane sends the jet along that exhaust direction. "Export sheet" writes the same table as timestamped Markdown and CSV into `exports/`. The Fusion coordinates use the frame and origin recorded in the scenario (`frame.cad_origin`, the reference point of the import), so they land on the dashboard's own coordinates.

### Airflow animation

The 3D view animates particles that follow the air into each duct, through the foil channel to the pressure point and out along the deflected exhaust. Colour and speed follow the jet intensity at hover using the CFD "turbo" scale: blue is idle, red is the fan-curve maximum; the legend shows the thrust and the momentum-theory jet velocity (T = rho A v squared with the 80 mm duct area) at the two ends. Change a foil angle and the exhaust turns with it. This is a kinematic illustration of where the air goes and how hard each motor works, not a CFD solution; it does not model mixing, losses or interaction between jets.

### Foil sweep: find the most efficient deflection automatically

The sweep panel (right column, below the metrics) tries every foil deflection in a grid, keeps the geometries that can hover level and steer every controlled axis, and ranks them by hover power. Choose the deflection range and step, the grouping (one angle for both foils, a segmented foil with one angle per motor pair, or left and right independent), the minimum hover headroom and minimum yaw authority, then click run. Each row shows the deflections outer to inner, hover power, headroom, yaw and roll authority and the score; grey rows are infeasible with the reason beside them. Click apply to load a row into the geometry panel and the 3D view. For scenarios without foils the same panel sweeps raw fan tilt. From the shell:

```bash
uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json --tilts 45,90,135 --grouping per_pair --min-headroom 0.05 --csv exports
```

### Control checks: authority the pilot actually gets on each axis

Torque in N m does not tell you whether a geometry is flyable, so every metrics call now carries a `control` block (Control group in the right panel) and the sweep can filter and rank on it. Per axis it reports the angular acceleration the attainable torque gives at hover (torque over the inertia about that axis, rad/s^2), the time to 10 degrees of bank from rest, the share of that torque reachable before PX4 has to desaturate a fan (beyond it axes start to couple), and the fore-aft or lateral force that leaks out when 20 percent of the axis is commanded, as a fraction of weight. It also states the rate-loop bandwidth the fan spool lag allows. The sweep panel takes minimum roll, pitch and yaw accelerations (defaults 8, 8 and 2.5 rad/s^2: a heavy multirotor that still feels crisp reaches 10 degrees of bank in about 0.2 s) and a maximum coupling, marks the weakest axis of each row, and "rank by most control authority" orders rows by the weakest axis over its requirement, penalised by coupling and surge. From the shell:

```bash
uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json --grouping per_pair --tilts 45:150:15 --rank-by control --min-roll-accel 8 --min-pitch-accel 8 --min-yaw-accel 5 --max-coupling 0.3 --min-headroom 0.1
```

**Hover attitude as a design parameter.** `frame.hover_pitch_deg` (nose-up degrees) is the attitude the airframe holds in hover; PX4's body frame, where the `CA_ROTOR*` geometry lives, is that hover frame, so every effective force point and thrust axis is rotated about +Y by it before allocation (`Scenario.hover_pos` / `hover_axis`), and the Gazebo and HITL models carry the CAD mesh rotated the same way. Hovering nose-up by t is equivalent, for the wing jets, to a level hover with each foil deflected t degrees further. The sweep takes a list of attitudes (`hover pitch°` in the panel, `--hover-pitch=-30:30:15` on the shell) and evaluates every geometry at each; rows show `@+15` for a pitched hover. On the CAD layout the as-built 45 degree foils trim at no attitude (wing and nose fans always disagree in direction), but pitched hovers open cheaper trims: 135/45/90/45 at +15 degrees nose-up hovers at 11.0 kW with roll 21.7, pitch 10.9 and yaw 7.1 rad/s^2, against 12.5 kW for the best level set with the same yaw requirement. Attitudes beyond about 15 degrees cost roll authority and headroom quickly.

What it found on the CAD layout (box-placeholder inertia, so compare rows relatively): pitch is the weakest axis of every hovering candidate and stays between 11 and 13 rad/s^2 whatever the foils do, because pitch comes from the two centreline fans and their arm, not from the foils; the foils trade roll (13 to 24 rad/s^2) against yaw (4 to 8 rad/s^2). Requiring 5 rad/s^2 of yaw, the best set is 135/60/135/45 degrees outer to inner (roll 19.3, pitch 12.4, yaw 7.7 rad/s^2, hover 11.9 kW), saved as `scenarios/atlas_phase01_cad_control.json`. Off-axis coupling and surge leak are zero for every hovering set: within the linear region the ten fans give the allocator enough freedom to hold Fx and Fy at zero. The handling limit that no deflection can fix is the 150 ms fan spool, which caps the rate loop near 2.7 rad/s and forces soft attitude gains; measuring the real spool time on the rig is the single most valuable input for the controller sizing.

What the foil sweep says about the as-built aircraft (placeholder mass and fan curve, so read relatively):

- **One angle for both foils never hovers level except at 90 degrees.** At any other angle all eight redirected jets push the same way fore or aft and nothing cancels it (stock PX4 then zeroes the wing motors, the log 36 behaviour). At 90 degrees it hovers at 8658 W with headroom 0.64 and 28 N m of roll, but has no yaw at all.
- **A segmented foil trims and yaws.** Giving pairs opposite fore-aft directions (some below 90, some above) cancels the fore-aft force and gives yaw from the differential. Of the 81 combinations of 45/90/135 degrees per pair, 43 are controllable. Cheapest hover: 90/90/135/45 degrees outer to inner at 9583 W with 7.5 N m yaw and headroom 0.53. Most yaw per kilowatt: 135/45/45/135 at 11519 W with 22.8 N m yaw. This needs the foil to bend differently along the span.
- **Left and right foils at different angles do not trim.** That cancels the fore-aft force but leaves a net yaw moment; it is a yaw control input, not a hover geometry.

Azimuth modes: `forward` and `aft` vector every wing fan the same way in the fore-aft plane (the as-built vehicle has all eight at 45 degrees forward); `alternating`, `outer_fwd_inner_aft` and `outer_aft_inner_fwd` give pairs opposite fore-aft directions; `inward` and `outward` tilt in the lateral plane.

What the sweep found on the CAD layout (placeholder mass and fan curve, so read the numbers relatively):

- **All wing fans vectored the same way (forward or aft, any angle) has no level-attitude hover trim.** Only the two centreline fans point up, so nothing cancels the fore-aft force of the eight wing fans. Stock PX4 drives the wing fans to zero (this is the log 36 behaviour) and even a fully-actuated allocator finds no level trim. Flying it means the body pitches to cancel the force, which the static metrics do not model.
- **Opposite directions across pairs fix it and give large yaw.** With `alternating` (outer pair forward, next aft, and so on) and the same tilt on all pairs: 15 degrees hovers at 8825 W with 4.65 N m yaw, 30 degrees at 9376 W with 9.7 N m, 45 degrees at 10528 W with 15.2 N m, against 8658 W and no yaw at all for vertical fans. Yaw comes from differential fore-aft thrust at the wing span, and the net fore-aft force cancels between pairs. Per-pair grids find cheaper mixes such as 15/15/0/0.
- **One shared lateral (inward) tilt never works** because the eight fans sit on a straight line, which makes roll, yaw and lateral force linearly dependent (rank 4). Per-pair inward tilts such as 10/0/10/0 restore yaw, but only about 0.4 N m.
- A row marked "PX4 allocator collapsed" is one where the PX4 pseudo-inverse became numerically unstable (near-dependent rows, for example the KM yaw model on top of an inward tilt); treat those geometries as unusable on the real controller.

### Replace the two placeholders before trusting absolute numbers

Every scenario currently carries an amber "estimated" banner because two inputs are placeholders:

- **Mass**: `mass.total_kg` is 12.0 with `"estimated": true`. Set the measured mass (and later CG and inertia from CAD) and set `estimated` to false.
- **Fan curve**: `fan_curves.xfly80_3280.points` has only the manufacturer maximum (33.3 N, 2450 W at cmd 1.0) with a linear ramp. Replace with thrust-stand points `[{"cmd", "thrust_N", "power_W"}, ...]` (monotone in cmd) and set `estimated` to false. `CA_ROTORn_CT` in the exported params is the thrust at cmd 1.0 of this curve.

Until then, read ratios between axes and between scenarios, not absolute N, N m or W.

### CAD geometry from the Atlas Mass & CoG dashboard

`tests/fixtures/cog_dashboard_full_8.html` is the Fusion 360 dashboard export of `PHASE_0.1_ASSY`: 37 bodies with exact centroids, volumes and meshes, including the 10 XFly EDFs. `scripts/import_cad_dashboard.py` turns it into a scenario:

```bash
uv run --project backend python scripts/import_cad_dashboard.py tests/fixtures/cog_dashboard_full_8.html
```

This writes `scenarios/atlas_phase01_cad.json` (fan positions from the CAD, PX4 rotor numbering: 0 to 7 wing fans in pairs from the outside in, left first; 8 and 9 centreline fans, rearmost first) and `docs/cad_import_report.json` (per-fan CAD axes, residuals against the flown parameter file).

- **Frame**: Fusion +z is forward and -y is up (`--forward`, `--up`), the dashboard's own display frame. It matches the aircraft: the inner wing motors sit at the level of the front fans and each pair outward is one step lower, 50 mm down per 87 mm outward (the 30 degree offset). The flown parameter files have the vertical coordinates upside down relative to this; exports from here are correct.
- **Weights are not in the HTML.** The dashboard keeps them in the browser and exports them with its Export button as JSON (`weights_by_type` in grams). Pass that file with `--weights` to get the real total mass, CG and inertia tensor. Without it the reference point is the dashboard's own volume-weighted centroid of all bodies (not a mass CG) and the mass stays the placeholder, both flagged `estimated`.
- **CAD model in the viewer**: the import also writes `scenarios/<name>.glb` with every body mesh (FRD metres relative to the reference point) and records it in `meta.cad_model`; the 3D view draws it translucent behind the motors and thrust arrows (toggle in the legend).
- **Fan orientation is not taken from the CAD.** As modelled, the eight wing EDF ducts lie along the fore-aft axis and the two centreline EDFs are vertical; the mounts are adjustable, so tilt and azimuth come from the template scenario (`--template`, default all vertical). The as-modelled axes are listed in the report.
- **Known discrepancy**: the two centreline fans sit about 0.24 m further aft in this CAD than the flown parameter file assumed (`CA_ROTOR8_PX 0.45`, `CA_ROTOR9_PX 0.56` versus 0.21 and 0.32 from the CAD with the fitted CG). The exported params from `atlas_phase01_cad` use the CAD positions.

### Scenario JSON

Fields (see `PLAN.md` section 5 and `backend/tiltlab/scenario.py`): `meta`, `frame`, `mass`, `fans` (10 entries: `pos_frd_m`, `tilt_deg`, `azimuth_deg`, `spin`, `mirror_of`, `curve_ref`, `km`), `fan_curves`, `control` (`concept`, `ca_method`, `reaction_torque`, `px4_params_override`), `rig`, `environment`, `outputs`. Fan positions are metres in FRD relative to the CG; they came from the CA_ROTOR parameters measured off the CAD. Thrust direction is derived, never stored: `a = (sin t cos p, sin t sin p, -cos t)`.

### Gazebo harness

The **Gazebo** button in the Export panel (or `POST /api/export/gazebo`) writes `exports/gazebo/<scenario>/` with a gz sim model (SDF 1.9: airframe body with the scenario mass and inertia and the CAD glTF, one rotor link per fan at the foil pressure point pointing along the effective thrust, each driven by the `MulticopterMotorModel` plugin scaled to the fan's effective CT), a world with the sensor systems PX4's gz bridge needs, a PX4 posix airframe file (`4500_gz_<scenario>`, an id no stock PX4 airframe shares, because PX4 matches autostart files by numeric prefix and 4010 collided with `4010_gz_x500_mono_cam`) carrying the same `CA_ROTOR*` geometry, and a README with the install and launch steps for a PX4 v1.17 checkout on Linux (`make px4_sitl gz_<scenario>`). It is generated, not run here; the README lists what to check first. It models the rotors as point thrusters at the effective directions; the foil aerodynamics and the Coanda turning are baked into those directions, not simulated. One PX4 limit matters for ten motors: the gz bridge declares only 8 ESC channels (`SIM_GZ_EC_FUNC1..8`), so before building PX4 raise `__max_num_servos` in `src/modules/simulation/gz_bridge/module.yaml` from 8 to 12; otherwise Gazebo reports "You tried to access index N of the Actuator velocity array" for the extra motors. A second gz bridge limit aborts PX4 outright with ten motors: the ESC feedback callback copies every motor speed into an `esc_status` message that holds 8 entries, overrunning the stack (`__stack_chk_fail`), so the copy must be clamped to 8. The harness README and the WSL helper script (`../wsl/tiltlab_gazebo.sh`) apply both patches. PX4 spawns the vehicle into the world itself (as `<scenario>_0`), so the world file does not include it. The model carries the IMU, barometer, magnetometer and GPS (navsat) sensors of PX4's x500 model; without the GPS the estimator never initialises and PX4 refuses to arm. To fly: with QGroundControl connected (inside WSL2 broadcasts never reach Windows, so the airframe starts the GCS link aimed at the Windows host on UDP 14550 and QGC auto-connects; fallback is a manual UDP comm link with listening port 14551 and server `<WSL IP>:18570`), type `commander takeoff` and `commander land` in the PX4 console, or use QGC's Takeoff slider and virtual joystick. The CAD mesh is written as `airframe.stl` in the gz body frame (FLU); the GLB used by the web viewer keeps FRD vertices, which Gazebo would draw upside down. The airframe also carries PX4 controller gains sized from tiltlab's torque authority, the inertia and the fan spool lag (`px4_tuning`), because PX4's defaults, tuned for a small quad, flip this 12 kg EDF airframe within two seconds of lift-off; with them the per-pair foil set in `scenarios/atlas_phase01_cad_hover.json` took off, hovered within 4 degrees and landed in gz sim (see `docs/gazebo_flight_2026-09-10.md`).

### Hardware-in-the-loop with the Pixhawk (Gazebo Classic)

PX4 HITL runs over MAVLink HIL messages on the Pixhawk's USB port, which only the Gazebo Classic `mavlink_interface` plugin speaks (the new gz sim bridge has no HITL mode). The **HITL** button in the Export panel (or `POST /api/export/gazebo_hitl`) writes `exports/gazebo_hitl/<scenario>_hitl/`: a Gazebo Classic model mirroring PX4's `iris_hitl` with one rotor per fan at the effective direction, the sensor plugins, the MAVLink interface in HIL serial mode with ten control channels, a world, a QGroundControl parameter file that puts the Pixhawk into HITL with this geometry (`SYS_AUTOSTART 1001`, `SYS_HITL 1`, `CA_ROTOR*`, `HIL_ACT_FUNC1..10`), and a README with the full procedure. Two facts verified in the pinned PX4 tree: HITL needs the `pwm_out_sim` module (`rcS` starts `pwm_out_sim start -m hil` when `SYS_HITL` is set) and no fmu-v6x board configuration compiles it, so one custom firmware build with `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y` is required. Fans and ESCs must be unpowered for every HITL session.

## API

| Method and path | Purpose |
|---|---|
| `GET /api/health` | liveness and version |
| `GET /api/scenarios`, `GET /api/scenarios/{name}`, `POST /api/scenarios/{name}` | list, load, save scenarios |
| `POST /api/metrics` `{scenario, concept?, collective?, weights?}` | full metrics payload, about 15 ms |
| `POST /api/px4_params_preview` `{scenario, concept?}` | CA_* parameter lines |
| `POST /api/export/params` `{scenario, concept?, base?}` | write timestamped `.params` into `exports/` |
| `POST /api/export/csv` `{rows, stem}` | write metrics CSV |

`docs/v1_metrics_snapshot.md` holds the metrics of the three shipped scenarios as returned by the API.

## Working on the code

- Conventions are in `CLAUDE.md`: one module per session, `make test` green before each commit, no hard-coded CT, mass or positions (they come from the Scenario), PX4 behaviour ported from the pinned tree with file:line citations, golden-test tolerances never loosened.
- The PX4 port is documented in `docs/px4_allocation_port_spec.md`. Where the code differs from a paraphrase of PX4, the code is right and cites the source.
- Golden fixtures and the ground truth extracted from them are in `tests/fixtures/README.md`. Tests read parameters from the logs themselves, never from the `.params` files.
- Backend dependencies are declared once in `backend/pyproject.toml`; frontend dependencies in `frontend/package.json` (React 18, react-three-fiber 8, drei 9, zustand, Tailwind 4, vitest). Do not add a dependency without a line in `PLAN.md` section 3.
- Layout: `backend/tiltlab/{core,api,export,px4,cad}`, `frontend/src/{components,store.ts,api.ts,types.ts}`, `tests/` (pytest, fixtures), `scenarios/`, `docs/`, `scripts/`, `third_party/` (gitignored PX4 checkout).

## PX4 caveats, read before flying anything exported from here

- Stock PX4 multicopter controllers command body-Z thrust only (the thrust setpoint x and y are exactly zero in every golden log). Fully-actuated metrics are only reachable in flight with a patched position/attitude controller or an offboard controller. The `fully_actuated` parameter set alone does not make stock PX4 use lateral thrust.
- `CA_METHOD 2` (AUTO) resolves to sequential desaturation for the multirotor airframe, and that is the path the replica reproduces.
- HITL: no stock `px4_fmu-v6x` board config compiles `pwm_out_sim`, so stock v1.17.0 v6x firmware cannot run HITL. A custom firmware build is required. SITL uses the `none_iris` target (SYS_AUTOSTART 10016).
- Reaction torque (KM) defaults to 0 in the exported geometry. Fit it from a rig yaw test before relying on KM yaw.
- Before writing an exported `.params` to the vehicle, diff it against your backup: only `CA_ROTOR*`, and for the fully-actuated concept `CA_METHOD` and `FD_FAIL_*`, should change.

## Safety for any future HITL or bench session

Fans unpowered. Remove the ESC power leads or the main battery before connecting the Pixhawk over USB, uploading parameters or arming.

## Docker (untested on the development machine)

`docker compose up` builds the backend image with the frontend static build and mounts `./scenarios` as a volume; open `http://host:8000`. Docker is not installed on the machine this was developed on, so the compose file has only been checked for correctness.

## Fixtures

Golden logs and parameter files under `tests/fixtures` were copied 2026-09-09 from the QGroundControl Daily Logs and Parameters folders: Pixhawk 6X, PX4 v1.17.0 (git d6f12ad1c4), SYS_AUTOSTART 4001, CA_ROTOR_COUNT 10, CA_AIRFRAME 0, CA_METHOD 2, vehicle on the rig (roll and position locked, pitch and yaw free).
