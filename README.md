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

### Tilt sweep: find the most efficient geometry automatically

The Tilt sweep panel (right column, below the metrics) evaluates a grid of wing-fan tilt angles and ranks them by hover power among the candidates that keep control. Set the tilt range and step, the azimuth mode (inward, outward, forward, aft), the minimum hover headroom and minimum yaw authority, then click run. Each row shows the tilt of the four wing pairs (outer to inner), hover power, headroom, yaw and roll authority and the score; grey rows are infeasible and their tooltip says why. Click apply on a row to load that geometry into the fan table and the 3D view. The same sweep runs from the shell:

```bash
uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json --tilts 0,10,20,30,40 --per-pair --min-headroom 0.12 --csv exports
```

What the sweep found on the CAD layout: the eight wing fans sit on a straight line, so one shared tilt for all pairs makes roll, yaw and lateral force linearly dependent (rank 4 at every angle) and stock PX4 cannot separate roll from yaw. Tick "per pair" so each pair gets its own angle; alternating tilts such as 10/0/10/0 degrees (outer to inner) restore independent yaw at almost no hover-power cost. A candidate marked "PX4 allocator collapsed" is one where the PX4 pseudo-inverse became numerically unstable (near-dependent rows, for example with the KM yaw model on top of an inward tilt); treat those geometries as unusable on the real controller.

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

- **Frame**: Fusion +z is forward and +y is up by default (`--forward`, `--up`). That choice reproduces the flown CA_ROTOR lateral and vertical positions to within 5 mm. The dashboard's own display uses up = -y and calls the model "shown inverted"; if the vehicle is anhedral rather than dihedral, rerun with `--up -y`.
- **Weights are not in the HTML.** The dashboard keeps them in the browser and exports them with its Export button as JSON (`weights_by_type` in grams). Pass that file with `--weights` to get the real total mass, CG and inertia tensor. Without it the script fits a reference CG to the flown parameter file and keeps the placeholder mass, flagged `estimated`.
- **Fan orientation is not taken from the CAD.** As modelled, the eight wing EDF ducts lie along the fore-aft axis and the two centreline EDFs are vertical; the mounts are adjustable, so tilt and azimuth come from the template scenario (`--template`, default all vertical). The as-modelled axes are listed in the report.
- **Known discrepancy**: the two centreline fans sit about 0.24 m further aft in this CAD than the flown parameter file assumed (`CA_ROTOR8_PX 0.45`, `CA_ROTOR9_PX 0.56` versus 0.21 and 0.32 from the CAD with the fitted CG). The exported params from `atlas_phase01_cad` use the CAD positions.

### Scenario JSON

Fields (see `PLAN.md` section 5 and `backend/tiltlab/scenario.py`): `meta`, `frame`, `mass`, `fans` (10 entries: `pos_frd_m`, `tilt_deg`, `azimuth_deg`, `spin`, `mirror_of`, `curve_ref`, `km`), `fan_curves`, `control` (`concept`, `ca_method`, `reaction_torque`, `px4_params_override`), `rig`, `environment`, `outputs`. Fan positions are metres in FRD relative to the CG; they came from the CA_ROTOR parameters measured off the CAD. Thrust direction is derived, never stored: `a = (sin t cos p, sin t sin p, -cos t)`.

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
