# tiltlab

Browser app for the 10-fan EDF aircraft: tilt every fan (0 to 90 degrees, any azimuth),
see what the geometry costs and buys on all six axes with a faithful replica of the PX4
v1.17.0 control allocator, and export the result as PX4 parameters, CSV and Scenario JSON.
Frames are FRD body and NED world, SI units internally, degrees only in the UI.

## Lean v1 scope

Built: Scenario model (`scenarios/*.json`), effectiveness matrix and allocator ported line by
line from the pinned PX4 tree (`third_party/PX4-Autopilot`, tag v1.17.0, commit d6f12ad1c4),
fan curves, static metrics, the explorer frontend, and timestamped exports.

Deferred to later versions (do not expect them here): CAD/STEP import and mass properties
(M6), 6-DOF dynamics and the controller replica (M7), PX4 SITL/HITL in the loop (M8),
PDF report, Gazebo SDF, plots, mechanical angle sheet, A/B snapshot compare, gamepad input.

## Quick start (Windows 11, Git Bash)

Tools live in user space and are not on PATH in every shell, so source the env script first.

```bash
source scripts/env.sh          # adds uv, make (~/.local/bin) and node/npm to PATH
uv sync --project backend      # Python 3.11 managed by uv; ignore the "minor version link" warning
npm --prefix frontend install
make dev                       # backend on :8000 (uvicorn --reload) and Vite on :5173
make test                      # pytest (golden tests against tests/fixtures/*.ulg) + vitest
make lint                      # ruff + tsc --noEmit
```

In PowerShell use `. .\scripts\env.ps1` instead of `source scripts/env.sh`. Never use the
system Python 3.14; run Python only through `uv run --project backend ...`.

## Quick start (Linux, macOS)

Install `uv`, `make` and Node 20+, then run the same commands. `scripts/env.sh` is harmless
there. To refresh the pinned PX4 sparse checkout run `scripts/fetch_px4.sh`.

## Docker (untested on this machine)

`docker compose up` builds the backend image (frontend static build served by the backend)
and mounts `./scenarios` as a volume; open `http://host:8000`. Docker is not installed on the
development machine, so `make docker` has only been checked for correctness, not run.

## Exports (backend/tiltlab/export)

Plain functions, no FastAPI inside, so the API layer can call them directly. Every file is
named `YYYYMMDD_HHMM_<stem>.<ext>` in local time (`naming.timestamped_name(stem, ext, now)`).

- `params.export_params(scenario, concept=None, base_params_path=None, out_dir=".", now=None)`
  writes `<stamp>_<scenario>_<concept>_px4.params`. Only `CA_ROTOR_COUNT` and
  `CA_ROTORn_{PX,PY,PZ,AX,AY,AZ,CT,KM}` for the ten fans change (CT from the fan curve at
  cmd 1.0, KM from the reaction-torque toggle), plus the concept set: `stock` leaves
  `CA_METHOD` as in the base file; `fully_actuated` sets `CA_METHOD 0`, `FD_FAIL_P 0`,
  `FD_FAIL_R 0`. Everything else in the user's full parameter backup is preserved byte for
  byte, CRLF included. The base defaults to `latest_backup_params()` (the newest stamped
  `tests/fixtures/*.params`). A header comment block lists tilt and azimuth per fan and the
  fan curve with its estimated flag.
- `params.import_params_to_scenario(path, template_scenario)` reads CA_ROTORn_* back into a
  Scenario (mass, curves, rig and meta from the template). Export then import reproduces tilt,
  azimuth and positions to float32 resolution (gate G6, local half).
- `csv_export.export_metrics_csv(rows, out_dir, stem="metrics", now=None)`: one row per
  scenario/concept dict, nested metrics flattened to `a.b` columns.
- `csv_export.export_scenario_json(scenario, out_dir, now=None)`.

Over HTTP the same two writers sit behind `POST /api/export/params` (`{scenario, concept?, base?}`)
and `POST /api/export/csv` (`{rows, stem}`); both answer `{path}` and write into `exports/` at the
repo root (gitignored, override with `TILTLAB_EXPORTS_DIR`). The explorer's Export panel (bottom
of the metrics column) calls them for the current scenario and concept. Other endpoints:
`GET /api/scenarios`, `GET|POST /api/scenarios/{name}`, `POST /api/metrics`,
`POST /api/px4_params_preview`, `GET /api/health`; `GET /` serves `frontend/dist` when built.
`docs/v1_metrics_snapshot.md` holds the metrics table of the three shipped scenarios as returned
by `/api/metrics` on this machine.

## PX4 caveats (read before flying anything exported from here)

- Stock PX4 multicopter controllers command body-Z thrust only (`vehicle_thrust_setpoint`
  x and y are exactly zero in every golden log). Fully-actuated metrics are only reachable in
  flight with a patched `mc_pos_control`/attitude controller or an offboard controller; the
  `fully_actuated` parameter set alone does not make stock PX4 use lateral thrust.
- `CA_METHOD 2` (AUTO) resolves to sequential desaturation for the multirotor airframe; the
  allocator replica reproduces exactly that path against the logs.
- HITL: no stock `px4_fmu-v6x` board config enables `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM`,
  so stock v1.17.0 v6x firmware cannot run HITL. A custom firmware build with that option
  enabled is required. SITL uses the `none_iris` target (SYS_AUTOSTART 10016).
- Fan curve: until thrust-stand data is entered the XFly 80 curve is the manufacturer maximum
  only; every metric and the params header carry the "estimated" flag.
- Reaction torque (KM) defaults to 0. Fit it from a rig yaw test before relying on KM yaw.

## Safety for any future HITL or bench session

Fans unpowered. Remove the ESC power leads or the main battery before connecting the Pixhawk
over USB, uploading parameters or arming for a HITL run. Check the exported `.params` diff
against your backup (only CA_ROTOR*, and for fully_actuated CA_METHOD/FD_FAIL_*, should
change) before writing it to the vehicle.

## Fixtures

Golden logs and parameter files under `tests/fixtures` were copied 2026-09-09 from
`C:\Users\stefa\Documents\QGroundControl Daily\{Logs,Parameters}`: Pixhawk 6X, PX4 v1.17.0
(git d6f12ad1c4), SYS_AUTOSTART 4001, CA_ROTOR_COUNT 10, CA_AIRFRAME 0, CA_METHOD 2, vehicle
on the rig (roll and position locked, pitch and yaw free). Ground truth and the rule "read
parameters from the log, never from the .params files" are in `tests/fixtures/README.md`.
