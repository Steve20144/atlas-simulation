---
name: pixhawk-debug
description: Debug a flight or bench problem on the Atlas 10-EDF Pixhawk 6X Pro (no throttle response, twitchy thrust, would not arm, HIL stuck on, motors uneven) by pulling the latest .ulg from the board through Vectra, triaging it with scripts/flight_debug.py, and reading the board live over USB. Use whenever the user says the Pixhawk is connected and asks to debug, diagnose or explain something that happened on the aircraft or the bench.
---

# Pixhawk flight-data debugger

Everything here was verified on this machine on 2026-09-12 to 14 against PX4 v1.17.0 (commit
d6f12ad1c4) on the Pixhawk 6X Pro. Repo: `C:\Users\stefa\Documents\Utopia Labs\vibe-coded`.
Read-only by default: this skill reads the board and the log. It never arms, never writes a
parameter and never reboots unless the user asks for that step in so many words.

## 0. What the user gives you

A symptom ("no throttle response", "went crazy when I touched the stick", "would not arm",
"HIL keeps coming back") and "I connected it". Everything else you find out yourself. Ask
nothing before step 4 unless the port is blocked.

## 1. Shell setup (every new shell)

```bash
cd "C:/Users/stefa/Documents/Utopia Labs/vibe-coded" && source scripts/env.sh
```

`uv`, `make`, `node` are not on PATH otherwise. Always run Python as
`uv run --project backend python ...`. Never the system Python.

## 2. Find the board and who owns the port

```bash
powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames()"
tasklist | grep -iE "QGroundControl|MissionPlanner"
usbipd list | grep -i 3185
```

- No port and no `3185:0035` in `usbipd list`: the board is not on USB. Say so and stop.
- Port exists but `QGroundControl.exe` runs: Windows lets one program hold a COM port. Ask the
  user to close QGC (or disconnect its link). Do not retry until they confirm.
- `usbipd` shows `Attached`: the port belongs to WSL (a HITL session). Stop that session first
  (Vectra: Launch Gazebo, Stop) or `usbipd detach --busid <BUSID>`.
- The board reboots itself off USB for about 8 s after a `reboot`; poll the port list.

The port is normally `COM3`. The CLI does not accept `auto`; pass the real name.

## 3. Pull the latest flight log

Preferred, through Vectra's backend (it locks the port, backs nothing up, needs no QGC):

```bash
curl -s -m 5 http://localhost:8000/api/health || echo backend-down
```

If the backend is down, start it with the Browser pane tool `preview_start` name
`vectra-backend` (never with Bash). Then:

```bash
curl -s -m 1200 -X POST http://localhost:8000/api/board/pull_log -H "content-type: application/json" -d '{"port":"COM3"}'
```

The answer carries `path` (under `exports/logs/`), `log_id`, `num_logs`, `size`, `time_utc`
and `url`. Newest means highest GPS time, then highest id, so a bench session without GPS
never outranks a real flight. USB moves about 50 kB/s: a 20 MB log takes seven minutes, so
use `run_in_background` on the Bash call for anything over a few MB and wait for the
notification. Without the backend:

```bash
uv run --project backend python scripts/px4_board.py --dev COM3 pull-log exports/logs
```

If the user means a flight from a specific day, list first and pick by hand: the LOG_ENTRY
`time_utc` is seconds since the epoch; `flight_debug.py` prints the date of the file name.

## 4. Triage the log

```bash
uv run --project backend python scripts/flight_debug.py "exports/logs/<file>.ulg" --json "C:/Users/stefa/AppData/Local/Temp/vectra_flight.json"
```

Read the sections in this order and match the symptom:

| Symptom | Where to look in the report | Usual cause on this aircraft |
|---|---|---|
| Armed but no motor response | `motors`: motors at 0 for most of the armed time; `allocator_armed.torque_achieved_pct` near 0 | Infeasible geometry pushed to the board: the PX4 allocator outputs 0 to every motor. Check the scenario in Vectra shows an exact stock hover before pushing. |
| Sticks ignored, would not arm | `arming.rc_calibration_in_progress_ever`, warnings "Arming denied: calibrating" | QGC Radio calibration started and QGC closed. Reboot clears it. |
| HIL stays on after setting SYS_HITL 0 | `log.params.SYS_AUTOSTART` = 1001, `hil_state_max` = 1 | The HIL airframe script sets SYS_HITL 1 at every boot. Use HIL off in Vectra (restores SYS_AUTOSTART and the rest), then reboot. |
| Thrust jumps at the first millimetre of stick | `stick_to_thrust.thrust_per_stick_below_40pct` > 1.5, `MPC_THR_HOVER` | Stabilized rescales the stick so mid stick = hover thrust. Lower MPC_THR_HOVER or test in Acro. |
| Idle thrust too high | `thrust_at_zero_stick` > 0.15, `MPC_MANTHR_MIN`, `RC3_MIN`/`RC3_TRIM` | Manual minimum thrust or RC3 calibration. |
| Fights the bench / tilts on its own | `attitude.pitch_deg_disarmed_mean` versus `SENS_BOARD_Y_OFF` | The hover pitch offset makes level read as nose-up; expected with hover_pitch_deg != 0. |
| One side spins, other side dead | `actuator_motors_armed` per motor, `actuator_outputs_armed` pulse widths, `PWM_MAIN_FUNC*` | Output mapping or allocator asymmetry. Compare with `exports/board/*_before.params`. |
| Preflight fail / no arm | `events.warnings_and_errors` | Read the message; `CBRK_SUPPLY_CHK 0` on USB power gives "system power unavailable". |
| Wobble / oscillation in hover | run `scripts/hover_report.py <ulg>` as well | Rate loop ringing at saturation; see the docstring of hover_report.py. |

`findings` at the bottom lists the signatures the script matched. Treat them as leads, not
verdicts: confirm each against the numbers above it.

## 5. Read the board live (read-only)

When the log is inconclusive or the symptom is "right now", read the board with NSH listener
commands through the shell. Batch several commands in one call; each takes about two seconds.

```bash
uv run --project backend python scripts/px4_board.py --dev COM3 status
uv run --project backend python scripts/px4_board.py --dev COM3 shell "listener vehicle_status 1" "listener actuator_armed 1" "listener input_rc 1" "listener manual_control_setpoint 1" "listener control_allocator_status 1" "listener actuator_motors 1" "px4io status" "pwm_out status" "param show SYS_HITL" "param show SYS_AUTOSTART" "param show CA_ROTOR*"
```

Write the output to the scratchpad with `> file` and grep it; never paste it whole. Fields
that matter: `arming_state` (1 standby, 2 armed), `nav_state` (15 Stabilized, 0 Manual, 10
Acro), `hil_state`, `rc_calibration_in_progress`, `pre_flight_checks_pass`, `input_rc.values`
(channel 3 is throttle, 5 the arm switch, 6 kill), `latest_arming_reason` (2 = RC switch),
`torque_setpoint_achieved`, `actuator_saturation` (2 upper, -2 lower).

For a moving picture of the thrust path, use the app: Pixhawk card, **Test thrust** (stick,
PX4 throttle, thrust setpoint, per-ESC pulse width at 5 Hz). Fans and ESCs unpowered, because
the outputs stay live. Stop it before other board operations, it holds the port.

Feed the board's CA_ROTOR values through the allocator replica when the geometry is suspect:
`vectra.core.geometry.rotors_from_px4_params` -> `compute_effectiveness_matrix` ->
`vectra.core.allocation.ControlAllocatorReplica(b[:, :n], ca_method=2)` -> `.step(torque, thrust)`.
All-zero motors for a pure thrust command means PX4 does the same.

## 6. Fixes live in Vectra, not in ad hoc writes

- Wrong geometry: load a scenario whose hover badge says exact, **Upload params** in the Pixhawk
  card (backs up to `exports/board/`, reads back). Or `POST /api/board/push` with the scenario.
- HIL stuck: **HIL off** (`POST /api/board/flight`) restores SYS_HITL, SYS_AUTOSTART, EKF2_EN,
  sensor presence, IMU calibration slots and gains from the flight backup; then **Reboot board**.
- RC calibration lock: **Reboot board** (`POST /api/board/reboot`).
- Any other single parameter (MC_AIRMODE, MPC_THR_HOVER, ...): **PX4 params** in the Metrics rail.
  Search by name or description, it reads the board value, type a new one, **write** (param set,
  save, read back). Backend: `GET /api/px4/params?q=`, `GET /api/board/param?name=`,
  `POST /api/board/param`. Catalogue from the pinned tree: `scripts/build_param_catalog.py`.
- Any write or reboot needs the user's go-ahead in the chat first, and the fans unpowered.

## 7. Report

Lead with the cause in one sentence, then the evidence (numbers from the report, file name and
log id), then the fix and what to check after it. Say what could not be verified. Put the
report file path in the message. Never claim the board is safe to fly from a log alone.
