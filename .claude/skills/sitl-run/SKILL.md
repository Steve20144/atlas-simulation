---
name: sitl-run
description: Run the Atlas 10-EDF aircraft in PX4 SITL with gz sim from a Vectra scenario, start to finish. Export the Gazebo harness, launch or reuse the sim in WSL Ubuntu-24.04, fly scripted hover tests, pull the ulog and read hover quality. Use whenever the user asks to fire up Gazebo, run SITL, fly the model, test a scenario in simulation, or get a log from the sim. For gain changes see the pid-tuning skill; for HITL and sweeps see vectra-sim.
compatibility: Windows 11 host with WSL2 Ubuntu-24.04 holding PX4 v1.17.0 at ~/PX4-Autopilot (px4_sitl_default built) and gz sim Harmonic. Repo at C:\Users\stefa\Documents\Utopia Labs\vibe-coded, reachable in WSL as ~/utopia/vibe-coded.
metadata:
  author: utopia-labs
  version: "1.0"
  verified: "2026-09-14"
---

# SITL run

One scenario in, one ulog out. Every command below was run on this machine on 2026-09-14.
Paths: repo `C:\Users\stefa\Documents\Utopia Labs\vibe-coded`, in WSL `~/utopia/vibe-coded`
(a symlink; the space in "Utopia Labs" breaks quoting through `wsl.exe`, so always use the symlink).

## 1. Shell rules that cost time when ignored

- Windows side: Git Bash, `cd "C:/Users/stefa/Documents/Utopia Labs/vibe-coded" && source scripts/env.sh`
  before any `uv run`.
- WSL side: always `wsl -d Ubuntu-24.04 -- bash -lc "<one command line>"`. Inside the quotes use
  `~` freely, but do not define shell variables (`P=...; bash $P ...`): the Windows shell strips
  `$P` before WSL sees it and every call degrades to `bash show`. Put multi-step logic in a script
  under `scripts/` of this skill and call the script.
- Scripts written from Windows need LF endings and the exec bit: `sed -i 's/\r$//' f.sh; chmod +x f.sh`,
  then `bash -n` them inside WSL.
- Anything that must outlive the tool call (the sim itself) is started by the launcher loop, not by
  a background Bash task: background tasks are SIGTERMed after the call ends. Scripted flights that
  finish within a few minutes are fine as `run_in_background` calls.

## 2. Is a sim already running

```bash
wsl -d Ubuntu-24.04 -- bash -lc "bash ~/utopia/vibe-coded/scripts/wsl/px4ctl.sh status; pgrep -af 'vectra_gazebo.sh --mode sitl' | head -2"
```

`status` lists `bin/px4`, `gz sim -s` (server), `gz sim -g` (GUI) and the vehicle's altitude. The
Vectra app's Launch Gazebo button and other sessions start sims here too. A running sim whose model
name matches the scenario you need, disarmed on the ground, is reusable as it is: do not stop it.
Check its geometry is current before trusting it: the harness is copied at launch, so an export
made after the launch is not in the sim (compare `ls -la --time-style=+%H:%M exports/gazebo/<name>/px4/airframes`
with the launch time from `ps -o etimes -p <px4 pid>`).

## 3. Export the harness

```bash
uv run --project backend python -c "import json,pathlib;from vectra.scenario import Scenario;from vectra.export.gazebo import export_gazebo;sc=Scenario.model_validate(json.loads(pathlib.Path('scenarios/<name>.json').read_text()));print(export_gazebo(sc, pathlib.Path('exports/gazebo'))['root'])"
```

Writes `exports/gazebo/<name>/{models,worlds,px4/airframes/<id>_gz_<name>,README.md}`. The airframe
carries the geometry (CA_ROTOR*), `SENS_BOARD_Y_OFF` = hover pitch, and the gains from
`px4_tuning()` overridden by `control.px4_params_override` of the scenario. Exports of the same
name overwrite each other and the user exports from the UI into the same tree: for an experiment,
save the scenario under a new name first (`scenarios/<name>_<variant>.json`) so nothing of theirs
is replaced.

## 4. Launch

The app is the one code path (`backend/vectra/api/gazebo_launch.py`); use it whenever the
backend is running (`curl -s localhost:8000/api/health`). It exports the harness, stops any sim
in that distro whoever started it, keeps PX4's stdin open, truncates the console log, reports
readiness from PX4's own `Ready for takeoff!` line and flags WSLg copy mode.

```bash
# launch (headless true skips the gz window; add it when WSLg misbehaves or for scripted runs)
curl -s -X POST localhost:8000/api/gazebo/launch -H "content-type: application/json" \
  -d "{\"scenario\": $(cat scenarios/<name>.json), \"mode\": \"sitl\", \"headless\": false}" | head -c 300
# poll until ready is true (about 60 to 90 s), then give the height estimate 10 s
curl -s localhost:8000/api/gazebo/status
curl -s -X POST localhost:8000/api/gazebo/takeoff      # arm, climb to 2.5 m, hold
curl -s localhost:8000/api/gazebo/probe                # armed, mode, height, allocator ok
curl -s -X POST localhost:8000/api/gazebo/land
curl -s -X POST localhost:8000/api/gazebo/log          # newest ulog -> exports/logs/sitl_<name>.ulg
curl -s -X POST localhost:8000/api/gazebo/stop
curl -s -X POST localhost:8000/api/gazebo/wsl_shutdown # only for WSLg copy mode; stops both distros
```

The same buttons are in the Metrics rail under Launch Gazebo (SITL, HITL, headless, Reset, Stop,
then Take off, Land, Pull log with a live probe line once PX4 is ready). Without the backend, the
scripts below do the same from a terminal.

Interactive, from PowerShell or Git Bash (opens the gz GUI window through WSLg):

```bash
wsl -d Ubuntu-24.04 -- bash -lc "bash ~/utopia/vibe-coded/scripts/wsl/vectra_gazebo.sh --mode sitl --yes --harness ~/utopia/vibe-coded/exports/gazebo/<name>"
```

`--yes` answers the package and saved-parameter questions (saved SITL parameters are cleared, so
the airframe's gains apply). Headless: `HEADLESS=1` before `bash` inside the quotes. The launcher
keeps a relaunch loop alive (reset support), so it must outlive the tool call. From an agent
session use the detached form, which reparents the launcher to init and feeds PX4's shell a FIFO
so it does not spin on EOF:

```bash
wsl -d Ubuntu-24.04 -- bash -lc "bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/launch_detached.sh <name> [--headless]"
wsl -d Ubuntu-24.04 -- bash -lc "bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/wait_ready.sh <name> 150"
```

`wait_ready.sh` blocks until `Ready for takeoff!` appears in `~/sitl_<name>.log` (or the launcher
dies), then prints the log highlights, the processes and the vehicle altitude (about z = 0.15 m
when spawned and settled). Give it another 10 s for the height estimate before flying. The app's
Launch Gazebo button does the same through the backend. Stop everything with
`vectra_gazebo.sh --stop` (also required before `launch_detached.sh` will start a new one).

A sim can wedge: `commander status` says Armed while `vehicle_status` says standby,
`actuator_armed` is minutes old and takeoff commands are accepted but nothing arms. The
commander task stopped publishing (seen after the app's reset rewound the gz clock while PX4
kept running). `px4ctl.sh reset` cannot recover it because the launcher's reset path also needs
the commander; stop and relaunch.

## 5. Fly a scripted hover test

```bash
wsl -d Ubuntu-24.04 -- bash -lc "bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/hover_run.sh --hover 40 --tag run1 [NAME VALUE ...]"
```

`scripts/hover_run.sh` sets the optional PX4 parameters live, takes off (auto takeoff to
`MIS_TAKEOFF_ALT`, 2.5 m), checks the vehicle climbed above 0.8 m after 15 s, hovers, lands, waits
for touchdown and the logger, and copies the newest ulog to `exports/logs/<tag>_<file>.ulg`. It
prints the log path last. Takes about hover time plus 45 s; run it with `run_in_background` and
wait for the notification. Exit 2 means no sim, 3 means it never climbed (read the last log).

Manual equivalents: `px4ctl.sh takeoff | land | param NAME VALUE | show NAME | log | reset | stop`.
`reset` restarts PX4 and respawns the vehicle at its start pose (fresh EKF and integrators, new
log); live parameter changes survive a reset but not a relaunch.

## 6. Read the result

```bash
uv run --project backend python scripts/hover_report.py exports/logs/<file>.ulg
```

Prints, for the last 20 s and the window before: roll, pitch, yaw peak to peak and dominant
frequency, setpoint swing, rate setpoint versus rate, normalised torque demand per axis (1 = the
allocator's full authority), position and altitude hold, mean motor command and the gains the log
ran with. Good hover here: attitude peak to peak under 1 deg, position under 0.3 m, torque demand
well under 1. Compare several runs at once with `scripts/tune_table.py` of the pid-tuning skill.
Write long outputs to the scratchpad and summarise; never paste a report into the conversation.

## 7. Where things go wrong

| symptom | cause | fix |
|---|---|---|
| every px4ctl step says `No such file or directory` | a `$VAR` inside the wsl command line was expanded on Windows | no variables in the command; call a script |
| `hover_run.sh` exit 3, vehicle at 0.15 m | no level hover trim for this geometry at this hover pitch, allocator outputs zero | run the metrics (`compute_metrics`) for the scenario; pick a pitch where hover is exact |
| lifts and flips within 2 s | default PX4 gains, or gains sized for a heavier placeholder inertia | gains from `px4_tuning` with real inertia; see pid-tuning |
| gz: `index N of the Actuator velocity array` | airframe not applied (CRLF, or id collision) | re-export; the launcher strips CR and removes stale vectra airframes |
| PX4 dies after `gz_bridge world:` | bridge patch for 10 motors missing | launcher applies it; rerun the launcher |
| grey GUI titled `[WARN:COPY MODE]` | WSLg fault | `wsl --shutdown` from PowerShell, relaunch |
| second launch will not start | previous `gz sim -s` holds the port | `vectra_gazebo.sh --stop`, then launch |
| log copied is an old file | logger not closed yet | `hover_run.sh` waits 6 s after touchdown; wait longer after a hard landing |

## 8. Cleaning up

`scripts/wsl_clean.sh` stops every simulator process in a distro, removes stale FIFOs and console
logs, optionally deletes the PX4 SITL ulogs (`--logs`, they are 20 to 30 MB per flight) and
purges the harness copies of scenarios that no longer exist (gz model, world, airframe with its
CMake line, Classic model and world):

```bash
wsl -d Ubuntu-24.04 -- bash -lc "bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/wsl_clean.sh --logs <old scenario> ..."
wsl -d Ubuntu-22.04 -- bash -lc "bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/wsl_clean.sh <old scenario> ..."
```

Follow it with `wsl --shutdown` when WSLg is in copy mode. On the Windows side the matching
leftovers are `exports/gazebo/<name>`, `exports/gazebo_hitl/<name>_hitl` and the scenario's
`.json` and `.glb`; check `grep -rl <name> tests docs frontend/src scripts` before deleting a
scenario, tests load several by name.

More failure signatures, HITL and sweeps: the `vectra-sim` skill.
