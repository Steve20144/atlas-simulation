# The simulation stack: what runs where, and why

Read this first. It is the map; `hitl_runbook.md` is the checklist, and `guide/gazebo_px4_guide.pdf`
is the from-scratch tutorial.

## One picture

```
 scenario JSON  ──► tiltlab (this repo) ──► harness export ──► WSL launcher ──► Gazebo + PX4
 scenarios/*.json    allocator replica,     exports/gazebo/<name>/       scripts/wsl/           ┌─ SITL: gz sim + PX4 SITL
                     metrics, sweeps,       exports/gazebo_hitl/<name>_hitl/   tiltlab_gazebo.sh     │  (Ubuntu-24.04, lockstep)
                     PX4 gain sizing        model, world, airframe/.params                       └─ HITL: Gazebo Classic + Pixhawk
                                                                                                    (Ubuntu-22.04, USB via usbipd)
```

Three ways to press the button, all ending in the same launcher:

| entry point | when |
|---|---|
| **App**: Metrics panel > Launch Gazebo > SITL / HITL / Stop | normal use; exports the current scenario and starts the launcher, shows its console tail |
| **Menu**: `make menu` (scripts/tiltlab_menu.py) | no browser; also exports, USB attach, firmware build, hover report, stop |
| **Shell**: `wsl -d <distro> -- bash ~/utopia/vibe-coded/scripts/wsl/tiltlab_gazebo.sh --mode <sitl|hitl> --harness <dir>` | scripting, debugging the launcher itself |

## Pieces

### tiltlab (Windows, `make dev` or the preview server on :8000)
- `backend/tiltlab/core/` replicates PX4's control allocator from the pinned v1.17.0 source and
  computes hover trim, authority, coupling and the sweep. `frame.hover_pitch_deg` rotates the
  airframe into the flight-controller frame, so "PX4 level" means "airframe at that pitch".
- `backend/tiltlab/export/gazebo.py` writes the gz sim harness and sizes PX4 gains (`px4_tuning`)
  from authority, inertia and fan lag. `control.px4_params_override` in the scenario wins over
  the rule for any non-`CA_` key: that is where a hand-tuned set is saved.
- `backend/tiltlab/export/gazebo_classic_hitl.py` writes the Gazebo Classic harness, the QGC
  `.params` for the board, the `fmu-v6x_hitl.px4board` firmware label and a README. The HITL set
  adds what the board needs to behave like SITL (see "HITL findings").
- `backend/tiltlab/api/gazebo_launch.py` is the app's launch/status/stop; it only picks arguments
  for the launcher.

### WSL launcher (`scripts/wsl/tiltlab_gazebo.sh`, run inside WSL)
Checks tools, installs the harness into the PX4 tree, applies the PX4 patches a 10-motor vehicle
needs (gz bridge ESC count 8 -> 12, `GZMixingInterfaceESC.cpp` clamp), normalises line endings,
detects the serial device, starts the QGC UDP relay for HITL, launches. `--stop` takes everything
down. `--build-firmware` builds and uploads `px4_fmu-v6x_hitl`. The same file is reachable as
`~/utopia/wsl/tiltlab_gazebo.sh` (symlink) for older commands. LF line endings are enforced by
`.gitattributes`; a CRLF bash script does not run.

### Board tool (`scripts/px4_board.py`, run inside WSL with pymavlink)
`status`, `shell`, `push <file.params>`, `verify <file.params>`, `pull-log <dir>`. It writes
parameters through the board's NSH (`param set`) because PX4 carries INT32 parameters byte-wise
in `PARAM_VALUE`: a float `PARAM_SET` corrupts every integer and a float read-back agrees with
itself (that mistake cost an afternoon; the test `test_px4_board.py` pins the decoding). While the
HITL sim owns the serial port the tool uses the sim's SDK UDP link, 14540.

### Helpers
`scripts/wsl/px4ctl.sh` (SITL: takeoff, land, param, log, stop), `scripts/wsl/qgc_udp_relay.py`
(WSL2 drops UDP broadcast, so the relay unicasts MAVLink to QGC on Windows),
`scripts/hover_report.py` (attitude, position, torque demand and gains from a ulog).

## SITL versus HITL, the part that matters

| | SITL (gz sim) | HITL (Gazebo Classic) |
|---|---|---|
| flight code | `px4_sitl_default` on the PC | `px4_fmu-v6x_hitl` on the Pixhawk (stock v6x has no `pwm_out_sim`; `default` + that module overflows flash by 22.9 kB, so a label drops the FW/VTOL modules) |
| time | PX4 clock **is** Gazebo's clock (`GZBridge.cpp:331`), physics never falls behind | board runs on its own clock, Gazebo must keep real time (`Lockstep is disabled`) |
| sensors | gz sim IMU/mag/baro/GPS through the gz bridge, EKF2 on the PC | `hil_state_level 1`: the board takes attitude and position from `HIL_STATE_QUATERNION`; `EKF2_EN 0` |
| config carrier | airframe file `NNNN_gz_<name>` | `.params` loaded on the board (`px4_board.py push`) |
| motors | `SIM_GZ_EC_*` 0..1000 rad/s | `HIL_ACT_FUNC1..10` = 101..110, `[0,1]` x `input_scaling 1000` |

Both carry the same 118 geometry and gain parameters; a test asserts the HITL file matches.

## HITL findings (2026-09-11), so nobody rediscovers them

1. **Rotor collisions**: the Classic model gave rotors collision cylinders; rotated by
   `hover_pitch_deg` they sat under the base box, the airframe rested on them and skittered, the
   accelerometer reported that and the EKF ran away (board roll -109 deg, parked). Removed.
2. **Tilted HIL accelerometer**: with the model level the board read ~25 deg of pitch, also with
   PX4's own `iris_hitl`. Not the geometry. Bypassed with `hil_state_level 1` (ground-truth state
   to the board) and `EKF2_EN 0` (otherwise ekf2 and the HIL handler both publish
   `vehicle_attitude` and `vz` flips between values).
3. **Sensor presence checks**: at state level 1 no `HIL_SENSOR` is sent, so `SYS_HAS_MAG 0`,
   `SYS_HAS_BARO 0`, and calibration slot 0 points at the receiver's SIM IMU (device id 1310988)
   because a flight-calibrated board otherwise reports its real ICM as "Accel Sensor 0 missing".
4. **Serial node moves**: a board reboot re-enumerates through usbipd as `/dev/ttyACM1`; the
   launcher now picks whichever single `ttyACM*` exists.
5. **Logging**: `SDLOG_BACKEND 0` on a flight-set-up board means rcS never starts the logger;
   the export sets `SDLOG_MODE 2`, `SDLOG_BACKEND 1`. Without a time source logs land in
   `sessNNN/` and list with `time_utc 0`: pick by id.
6. **Flight termination latches**: past 60 deg of roll PX4 terminates and only a board reboot
   clears it. Realistic, kept; `CBRK_FLIGHTTERM 121212` disables it for bench iteration.
7. **The real aircraft**: the IMU is bolted to the airframe and will read the hover pitch;
   `SENS_BOARD_Y_OFF` (deg) makes PX4's body frame the hover frame. It is applied to real
   sensors only and does nothing in HIL, so it is not in the HITL set.

## Where a new geometry goes
1. Sweep or set deflections in the app, Save under a new scenario name.
2. Launch SITL from the app; fly `commander takeoff` (or `px4ctl.sh takeoff`); read
   `hover_report.py` on the log; tune live with `px4ctl.sh param`; save the winners in
   `control.px4_params_override`.
3. Launch HITL from the app after the one-time board setup in `hitl_runbook.md`; the same gains
   ride along automatically.
