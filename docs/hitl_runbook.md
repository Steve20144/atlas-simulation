# HITL runbook: `atlas_phase01_cad_control` on the Pixhawk 6X

The HITL harness is the hardware twin of the SITL setup flown on 2026-09-10
(`docs/gazebo_flight_2026-09-10.md`): same scenario, same geometry, same controller gains.
The Pixhawk runs real PX4 firmware and flies a Gazebo Classic model over USB; the fans and ESCs
stay unpowered.

| | SITL | HITL |
|---|---|---|
| scenario | `scenarios/atlas_phase01_cad_control.json` | same |
| export | `exports/gazebo/atlas_phase01_cad_control` | `exports/gazebo_hitl/atlas_phase01_cad_control_hitl` |
| simulator | gz sim (Harmonic), Ubuntu-24.04 | Gazebo Classic 11, Ubuntu-22.04 |
| flight code | `px4_sitl_default` on the PC | `px4_fmu-v6x_hitl` on the board |
| config carrier | airframe `4615_gz_atlas_phase01_cad_control` | `px4/atlas_phase01_cad_control_hitl.params` (QGC) |

Regenerate both from the same scenario; the 118 shared parameters are identical by construction:

```bash
uv run --project backend python -c "import json,pathlib;from tiltlab.scenario import Scenario;from tiltlab.export.gazebo_classic_hitl import export_gazebo_classic_hitl as f;sc=Scenario.model_validate(json.loads(pathlib.Path('scenarios/atlas_phase01_cad_control.json').read_text()));print(f(sc, pathlib.Path('exports/gazebo_hitl'))['root'])"
```

Gains carried over unchanged (validated in gz sim, hover within 0.5 deg and 25 cm):
`MC_ROLLRATE_P 0.0985`, `MC_PITCHRATE_P 0.1541`, `MC_YAWRATE_P 0.2482`, I = 0.6 P, D = 0.05 P,
`MC_*_INT_LIM 0.15`, `MC_ROLL_P` = `MC_PITCH_P` = `MC_YAW_P` 0.762, `MPC_XY_VEL_P_ACC 0.571`,
`MPC_XY_P 0.358`, `MPC_Z_VEL_P_ACC 1.448`, `MPC_TILTMAX_AIR 20`, `MPC_THR_HOVER 0.484`,
`MC_AT_EN 0` (never run QGC Autotune on this airframe).
HITL adds `SYS_HITL 1`, `SYS_AUTOSTART 1001`, `HIL_ACT_FUNC1..10 = 101..110`,
`CBRK_SUPPLY_CHK 894281`, `UAVCAN_ENABLE 0`; it drops the SITL-only `SIM_GZ_*` and `MAV_0_BROADCAST`.

## Machine state, checked 2026-09-11

Ready: Ubuntu-22.04 (WSL2) with Gazebo Classic 11.10.2 and libgazebo-dev, PX4 v1.17.0 at
`~/PX4-Autopilot` with the `sitl_gazebo-classic` plugins built, `stefa` in `dialout`,
usbipd-win 5.3.0 on Windows with the Pixhawk already bound (it survives replug), and the ARM
toolchain (`arm-none-eabi-gcc` 10.3.1) installed by `Tools/setup/ubuntu.sh`.

Flashed 2026-09-11: `px4_fmu-v6x_hitl` (1.72 MB) is on the board with the HITL parameter set,
`EKF2_EN 0` and the HIL sensor-presence set; the harness runs `hil_state_level 1`.

## 0. Firmware with `pwm_out_sim` (once)

Stock v1.17.0 v6x firmware cannot run HITL: with `SYS_HITL` set, rcS runs `pwm_out_sim start -m
hil`, and no fmu-v6x board config compiles that module. Simply adding it to `default.px4board`
does not link either, measured on this tree:

```
region `FLASH' overflowed by 22884 bytes      FLASH: 1988964 B / 1920 KB = 101.16%
```

So the harness ships `px4/fmu-v6x_hitl.px4board`, a delta on `default.px4board` (merged by
`cmake/kconfig.cmake:50`) that adds `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y` and drops the
fixed-wing and VTOL modules a multirotor HITL never runs, exactly the set PX4's own
`multicopter.px4board` drops. That links at 1859824 B, 94.6% of flash, and leaves
`default.px4board` untouched for flight builds.

```bash
wsl -d Ubuntu-22.04 -- bash -lc "bash ~/utopia/vibe-coded/scripts/wsl/tiltlab_gazebo.sh --build-firmware --harness ~/utopia/vibe-coded/exports/gazebo_hitl/atlas_phase01_cad_control_hitl"
```

The launcher copies the label to `boards/px4/fmu-v6x/hitl.px4board`, runs `make px4_fmu-v6x_hitl`
and offers `make px4_fmu-v6x_hitl upload`. The board must be attached (step 1) and QGroundControl
closed for the upload. Same target for 6X and 6X Pro. If the ARM toolchain is ever missing, run
`bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh` first (without `--no-nuttx`) and open a new shell.

## 1. USB passthrough into WSL

The board is a USB serial device on Windows; WSL2 needs it forwarded with usbipd-win. The WSL2 VM
is shared by both distros, so the device appears in Ubuntu-22.04 regardless of which distro is
named, but name it anyway (the default distro here is Ubuntu-24.04).

Plug in the Pixhawk over USB, then in PowerShell:

```powershell
usbipd list
```

Find the Pixhawk row (it enumerates as `USB Serial Device (COM<n>)`) and note its BUSID. If its
STATE is `Not shared`, bind it once from an **administrator** PowerShell (persists across reboots):

```powershell
usbipd bind --busid <BUSID>
```

Then attach it to WSL (normal PowerShell is enough; `--auto-attach` re-attaches after a replug or
a board reboot and keeps running in that window):

```powershell
usbipd attach --wsl Ubuntu-22.04 --busid <BUSID> --auto-attach
```

Confirm inside WSL:

```bash
wsl -d Ubuntu-22.04 -- bash -lc "ls -l /dev/ttyACM*"
```

`/dev/ttyACM0` owned by `root:dialout` is what the harness expects. A different number needs
`--serial /dev/ttyACM1` on the launcher. To give the board back to Windows (for QGC over USB, or
before a real flight): `usbipd detach --busid <BUSID>` — a rebooting board also drops off until
auto-attach picks it up again.

## 2. Load the parameters

1. Detach the board from WSL (step 1) so QGroundControl on Windows can open the COM port.
2. QGC > Vehicle Setup > Parameters > Tools > Load from file:
   `exports\gazebo_hitl\atlas_phase01_cad_control_hitl\px4\atlas_phase01_cad_control_hitl.params`
3. Reboot the board. `SYS_HITL` is read only at boot.
4. In QGC's MAVLink console: `listener vehicle_status` must show `hil_state: 1`, and
   `pwm_out_sim status` must report the module running. If either fails, the firmware from step 0
   is not the one on the board.
5. Close QGroundControl. Gazebo owns the serial port; QGC reconnects over UDP.
6. Re-attach the board to WSL (step 1).

## 3. Launch

Fans and ESCs unpowered. From PowerShell:

```powershell
wsl -d Ubuntu-22.04 -- bash -lc "bash ~/utopia/vibe-coded/scripts/wsl/tiltlab_gazebo.sh --harness ~/utopia/vibe-coded/exports/gazebo_hitl/atlas_phase01_cad_control_hitl"
```

HITL is the launcher's default mode. It copies the model and world into
`~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/`, strips CR, forces
`<qgc_addr>INADDR_ANY</qgc_addr>`, checks the serial device, starts `qgc_udp_relay.py` (WSL2 never
delivers the plugin's UDP broadcast to Windows) and runs `gazebo --verbose`. Expect
`Opened serial device /dev/ttyACM0`. Reopen QGC after the window is up; it connects over UDP 14550,
and the fallback is a manual UDP link, listening port 14551, server `<WSL ip>:18570`.

Arm and fly from QGC or the transmitter. `bash ~/utopia/vibe-coded/scripts/wsl/px4ctl.sh takeoff|land|log|param`
works the same as in SITL, and `scripts/hover_report.py` reads the copied ulog.

## Returning to flight configuration

Load the real flight parameter file, confirm `SYS_HITL` is 0, reboot, and check `hil_state: 0`
before powering anything.

## Known failure signatures

| symptom | cause | fix |
|---|---|---|
| `No valid data from Accel 0`, `hil_state: 0` | board did not boot with `SYS_HITL 1` | `param set SYS_HITL 1; param save; reboot`, re-attach USB, restart Gazebo |
| `system power unavailable`, `Battery unhealthy` | a saved `CBRK_SUPPLY_CHK 0` from flight setup | `param set CBRK_SUPPLY_CHK 894281` (the exported file sets it) |
| no `/dev/ttyACM*` in WSL | not attached, or the board rebooted | repeat `usbipd attach`, use `--auto-attach` |
| `ttyACM0 exists but cannot be opened` | not in `dialout` in this session | `wsl --terminate Ubuntu-22.04`, new shell |
| QGC stays "Disconnected" while Gazebo runs | relay not running or firewall | check `qgc_udp_relay.py` in the launcher output, then the manual UDP link above |
| second launch opens a grey or duplicate window, or Gazebo will not start again | `gzserver` from the previous run still holds the master port 11345, so the new client attaches to the old scene | the launcher now offers to stop it; by hand: `pkill -x gzclient; pkill -x gzserver` |
| board attitude 25 deg or more off the level model, arms into a lunge; PX4's own `iris_hitl` shows it too | Classic IMU plugin feeds the EKF a tilted acceleration; not the geometry, not `hover_pitch_deg` | harness runs `hil_state_level 1` with `EKF2_EN 0`: the board takes attitude and position from `HIL_STATE_QUATERNION` and ekf2 never publishes against it |
| `vz` alternating 0 and ~23 m/s, attitude garbage, after `hil_state_level 1` | ekf2 still running: two publishers on `vehicle_attitude` / `vehicle_local_position` | `EKF2_EN 0` (rcS:371 gates `ekf2 start` on it), in the exported .params |
| `Preflight Fail: Accel/Gyro Sensor 0 missing`, `barometer 0 missing`, `Found 0 compass` at state level 1 | no `HIL_SENSOR` is sent at that level (mavlink_interface.cpp:277, :303); calibration slots still name the real ICM | exported: `SYS_HAS_MAG 0`, `SYS_HAS_BARO 0`, `CAL_ACC0_ID`/`CAL_GYRO0_ID 1310988` (DRV_IMU_DEVTYPE_SIM) with identity calibration, slot 1 cleared |
| `Error opening serial device: set_option: Input/output error`, `Tx queue overflow`, no telemetry | board reboot re-enumerated through usbipd as `/dev/ttyACM1`; launcher pointed Gazebo at the stale ACM0 | launcher now uses the single `ttyACM*` present when the default is absent |
| integer params land as garbage (`HIL_ACT_FUNC1` = 1120534528, `SYS_AUTOSTART` 0) after a MAVLink param load | PX4 carries INT32 params byte-wise in `PARAM_VALUE`; writing them as floats corrupts them and a float read-back agrees with itself | load through the board shell (`param set`) or decode INT32 with struct; never trust a float read-back for ints |
| grey Gazebo window titled `[WARN:COPY MODE]` | WSLg lost its shared-memory channel at session start (`/mnt/wslg/weston.log`: `rdp_allocate_shared_memory ... Input/output error`, `use_gfxredir = 0`); nothing inside WSL fixes it | `wsl --shutdown` from Windows (closes every distro), then launch again |
