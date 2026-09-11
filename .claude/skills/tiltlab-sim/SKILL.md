---
name: tiltlab-sim
description: Operate the Atlas 10-EDF simulation stack: sweep foil deflections and hover attitude in tiltlab, export Gazebo SITL or Pixhawk HITL harnesses, launch them in WSL, tune PX4 parameters live, read hover quality from logs. Use for anything involving tiltlab sweeps, Gazebo, PX4 SITL/HITL, QGroundControl links or controller gains on this aircraft.
---

# tiltlab simulation operator

Everything below was verified on this machine on 2026-09-10/11. Paths are fixed: the repo is
`C:\Users\stefa\Documents\Utopia Labs\vibe-coded` (in WSL: `~/utopia/vibe-coded`, a symlink that
exists in both distros because the space in "Utopia Labs" breaks quoting through `wsl.exe`).
The helper scripts live in `~/utopia/vibe-coded/scripts/wsl/` (outside the repo).

## The stack in one picture

1. **tiltlab** (this repo): scenario JSON -> PX4 allocator replica -> metrics, control checks, sweeps.
   Backend `make dev` on :8000, UI at http://127.0.0.1:8000 (rebuild with `npm --prefix frontend run build`
   after frontend changes; the backend serves `frontend/dist`).
2. **Exporters** (`backend/tiltlab/export/`): `gazebo.py` writes a gz sim model, world and PX4 airframe
   (`exports/gazebo/<scenario>/`); `gazebo_classic_hitl.py` writes the Gazebo Classic model and a QGC
   `.params` file for the real Pixhawk (`exports/gazebo_hitl/<scenario>_hitl/`). Both carry the PX4
   controller gains from `px4_tuning()`.
3. **WSL launcher** `~/utopia/vibe-coded/scripts/wsl/tiltlab_gazebo.sh`: installs a harness into the PX4 v1.17.0 checkout
   (`~/PX4-Autopilot`), applies the required PX4 patches, clears saved params, launches.
   SITL runs in **Ubuntu-24.04** (gz sim Harmonic). HITL runs in **Ubuntu-22.04** (Gazebo Classic 11).
4. **Live control** `~/utopia/vibe-coded/scripts/wsl/px4ctl.sh`: takeoff, land, set/show parameters, copy logs, stop.
5. **Log analysis** `scripts/hover_report.py`: attitude, setpoints, torque demand, position hold, gains.

## Workflow A: find a geometry (sweep)

Run from the repo root in Git Bash after `source scripts/env.sh`:

```bash
uv run --project backend python scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json \
  --grouping per_pair --tilts 45,60,90,120,135,150 --hover-pitch 0,10,20 \
  --rank-by control --min-headroom 0.1 --min-roll-accel 8 --min-pitch-accel 8 --min-yaw-accel 5 --max-coupling 0.3
```

- Angles are foil deflections per motor pair, outer to inner; `--hover-pitch` is the hover attitude
  list (nose-up degrees; use `--hover-pitch=-20:20:10` when the list starts with a minus).
- `rank_by control` = weakest axis' angular acceleration over its requirement, penalised by coupling.
  Grey rows carry the reason. Keep the grid under 5000 candidates (the cap).
- Same thing in the UI: Foil sweep panel, "apply" a row, then Save under a new scenario name so the
  export gets its own directory (exports of the same name overwrite each other).
- Known results: pitch authority is 11 to 13 rad/s^2 on every hovering set (set by the nose fans, not
  the foils); the as-built 45 deg foils trim at no attitude; best control set 135/60/135/45 level;
  cheaper hover 135/45/90/45 at +15 deg nose-up.

To turn a CLI winner into a scenario, copy the pattern in `docs/gazebo_flight_2026-09-10.md`
(`apply_deflections` + `frame.hover_pitch_deg`, write `scenarios/<name>.json`).

## Workflow B: export and fly SITL (gz sim, no hardware)

```bash
uv run --project backend python -c "import json,pathlib;from tiltlab.scenario import Scenario;from tiltlab.export.gazebo import export_gazebo;sc=Scenario.model_validate(json.loads(pathlib.Path('scenarios/atlas_phase01_cad_control.json').read_text()));print(export_gazebo(sc, pathlib.Path('exports/gazebo'))['root'])"
```

Launch from PowerShell (opens Gazebo GUI and the PX4 console in that window):

```bash
wsl -d Ubuntu-24.04 -- bash -lc "bash ~/utopia/vibe-coded/scripts/wsl/tiltlab_gazebo.sh --mode sitl --harness ~/utopia/vibe-coded/exports/gazebo/atlas_phase01_cad_control"
```

Answer yes to clearing saved SITL parameters. Wait for `Ready for takeoff!` plus ~10 s (height
estimate), then `commander takeoff` in the PX4 console, `commander land` to finish. From another
shell you can drive it without the console: `wsl -d Ubuntu-24.04 -- bash ~/utopia/vibe-coded/scripts/wsl/px4ctl.sh takeoff`.

Headless variant for scripted tests: prefix `HEADLESS=1` inside the quotes; watch via QGC or
`gz topic -e -t /<model>_0/command/motor_speed`.

## Workflow C: HITL (real Pixhawk 6X, your RC transmitter)

Prerequisites once: firmware with `pwm_out_sim` (`--build-firmware`), usbipd-win on Windows
(`usbipd list; usbipd bind --busid <id>; usbipd attach --wsl --busid <id> --auto-attach`), user in
`dialout`. Every session: load `exports/gazebo_hitl/<name>_hitl/px4/<name>_hitl.params` in QGC over USB
(sets SYS_HITL 1, CA_ROTOR*, HIL_ACT_FUNC1..10, the gains, CBRK_SUPPLY_CHK 894281), reboot, confirm
`listener vehicle_status` shows `hil_state: 1` (HIL is set only at boot), close QGC, then:

```bash
wsl -d Ubuntu-22.04 -- bash -lc "bash ~/utopia/vibe-coded/scripts/wsl/tiltlab_gazebo.sh --harness ~/utopia/vibe-coded/exports/gazebo_hitl/atlas_phase01_cad_control_hitl"
```

The script starts `qgc_udp_relay.py` so QGC on Windows reconnects over UDP. Fans and ESCs unpowered.
Before a real flight reload the flight parameter file and confirm `SYS_HITL` is 0.

## Workflow D: tune parameters live, then bake them in

1. Change while hovering: `bash ~/utopia/vibe-coded/scripts/wsl/px4ctl.sh param MC_PITCHRATE_P 0.15 MC_PITCH_P 0.8`
   (repeat name value pairs). Land and take off again so integrators restart.
2. Measure: `bash ~/utopia/vibe-coded/scripts/wsl/px4ctl.sh log` copies the newest ulog to `exports/logs/`, then
   `uv run --project backend python scripts/hover_report.py exports/logs/<file>.ulg`.
   Good hover on this airframe: attitude pk-pk under 1 deg, position under 0.3 m, torque demand well below 1.
3. Bake: edit `px4_tuning()` in `backend/tiltlab/export/gazebo.py` (rule-based, scales with fan lag,
   authority and inertia), run `uv run --project backend pytest tests -q`, re-export, relaunch.
   Live values never survive a relaunch by themselves.

Sizing rule that is known to hover cleanly (fan lag 150 ms): rate crossover 1/(3.5 x lag), rate P =
crossover / (authority / inertia), I = 0.6 P, D = 0.05 P, integrator limits 0.15, attitude P =
crossover / 2.5 on all three axes, MPC_XY_VEL_P_ACC 0.75 x attitude bandwidth, MPC_XY_P 0.47 x,
MPC_Z_VEL_P_ACC 1.9 x, tilt limit 20 deg, `MC_AT_EN 0`.

## Failure signatures (all seen here)

| symptom | cause | fix |
|---|---|---|
| Gazebo: `index N of the Actuator velocity array which is of size 4` | airframe not applied: CRLF file, or autostart id collides with a stock PX4 airframe (rcS sources `<id>_*`, last wins) | exporter writes LF and derives the id from the scenario name; script strips CR and removes stale tiltlab airframes |
| PX4 dies right after `gz_bridge world: ...` (signal 6) | gz bridge `esc_status` holds 8 entries, 10 motors overrun it | script patches `GZMixingInterfaceESC.cpp` and `max_num_servos` 8 -> 12 |
| motors stay at zero after takeoff, allocator reports thrust unallocated | geometry has no level hover trim (e.g. all foils 45 deg) | pick a sweep row that is feasible |
| lifts then flips within 2 s | PX4 default gains on a 12 kg, 150 ms-fan airframe | gains from `px4_tuning` |
| slow 0.25 Hz roll/pitch swing to the tilt limit, metres of wander | position loop faster than attitude loop | MPC_* scaled below attitude bandwidth (in `px4_tuning`) |
| axes ring one after another, gains in log 3 to 4x the airframe's | PX4 autotune ran (QGC tuning page) | `MC_AT_EN 0`; never run Autotune here |
| QGC "Disconnected" though PX4 runs | WSL2 drops UDP broadcast to Windows | airframe starts the GCS link at the Windows host IP; fallback: QGC manual UDP link, listening port 14551, server `<WSL ip>:18570` |
| second Gazebo launch shows a grey or duplicate window, or will not start | `gzserver` from the previous run still holds the master port 11345 | `pkill -x gzclient; pkill -x gzserver` (the launcher asks before doing it) |
| HITL board tilted 25 deg+ against a level model, lunges on arming | Classic IMU plugin gives the EKF tilted acceleration | harness: `hil_state_level 1` + `EKF2_EN 0`; board takes `HIL_STATE_QUATERNION` |
| HITL `Preflight Fail: Accel Sensor 0 missing` / `0 compass` / `barometer 0 missing` | no HIL_SENSOR at state level 1; CAL slots name the real ICM | exported `SYS_HAS_MAG/BARO 0`, `CAL_ACC0_ID`/`CAL_GYRO0_ID 1310988` |
| HITL `set_option: Input/output error`, `Tx queue overflow` | board came back as `/dev/ttyACM1` after reboot | launcher auto-selects the one `ttyACM*` present |
| int params garbage after MAVLink param load | INT32 is byte-wise in PARAM_VALUE | `param set` via the board shell, or struct-decode ints |
| grey Gazebo window titled `[WARN:COPY MODE]` | WSLg shared-memory channel failed (`/mnt/wslg/weston.log`: `rdp_allocate_shared_memory ... Input/output error`, `use_gfxredir = 0`) | `wsl --shutdown` from PowerShell, relaunch |
| HITL: `No valid data from Accel 0`, `hil_state: 0` | board not booted with SYS_HITL 1, or commander restarted | `param set SYS_HITL 1; param save; reboot`, re-attach USB, restart Gazebo |
| HITL: `system power unavailable`, `Battery unhealthy` | saved `CBRK_SUPPLY_CHK 0` from flight setup | `param set CBRK_SUPPLY_CHK 894281` (now in the exported file) |

## Conventions when editing this repo for the sim

- Frames: FRD body, `frame.hover_pitch_deg` rotates airframe vectors into the flight-controller frame;
  gz body is FLU via (x, -y, -z). Never hard-code geometry; it comes from the Scenario.
- Keep `make test` green (backend pytest + vitest) and commit with a one-line message; the user has
  asked for pushes to `main` after each finished step.
- Tokens are precious to this user: read files by section, avoid dumping logs, prefer one combined
  shell call per step, and state the token cost of long tasks before starting them.
