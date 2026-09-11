# Gazebo end-to-end flight, 2026-09-10

Goal: take the Atlas phase-01 CAD scenario through tiltlab's foil sweep, export the winner as a
Gazebo (gz sim) harness and fly it with PX4 v1.17 SITL in WSL2, without a ground station.

## Foil sweep

`scripts/sweep_tilt.py scenarios/atlas_phase01_cad.json --variable foil --grouping per_pair
--tilts 45:150:15 --rank-by score --min-headroom 0.1` evaluated 4096 per-pair deflection sets in
64 s; 1619 hover. A single shared deflection (0 to 180 deg in 5 deg steps) never hovers: with all
eight jets turned the same way the redirected thrust always has a net fore-aft component the two
vertical nose fans cannot cancel, which is also what PX4's allocator found in Gazebo (motor
commands stayed at zero for the all-45-degree set).

Best by score, outer pair to inner pair: **135, 60, 120, 60 deg**. Outer pairs blow forward-up
(jet turned past vertical), inner pairs aft-up, so the fore-aft components cancel. tiltlab: hover
collective 0.45, headroom 0.51, roll 19.3 N m, pitch 18.4 N m, yaw 15.7 N m, all attainable,
hover power 11.0 kW with the estimated fan curve. Saved as `scenarios/atlas_phase01_cad_hover.json`.

## What had to change in the harness before it flew

1. PX4's gz bridge has 8 ESC channels (`SIM_GZ_EC_FUNC1..8`) and its ESC feedback callback
   overruns an 8-entry array with 10 motors, aborting PX4 (`__stack_chk_fail`). The WSL script
   patches both (`__max_num_servos` 8 to 12, clamp in `GZMixingInterfaceESC::motorSpeedCallback`).
2. Autostart id 4010 collided with `4010_gz_x500_mono_cam`; ids are now derived from the scenario
   name (4500 to 4899).
3. Windows line endings in the airframe file made every `param set-default` fail silently.
4. The world included the vehicle although PX4 spawns it; the CAD GLB (FRD vertices) rendered
   upside down; the model lacked the navsat sensor PX4 needs to arm.
5. WSL2 drops UDP broadcasts to Windows, so the airframe starts the GCS MAVLink link aimed at the
   Windows host directly.
6. With PX4's default MC_* gains the vehicle lifted off and flipped within two seconds. The gains
   fit a small quad (about 130 rad/s^2 per unit normalised torque, 12 ms motor spool). This
   airframe has 7 to 21 rad/s^2 per unit with a 150 ms fan spool, so the rate loop was slower
   than the attitude loop above it. The airframe now carries gains sized from tiltlab's torque
   authority, the inertia and the fan lag (see `px4_tuning` in `tiltlab/export/gazebo.py`), a
   zero idle command with `THR_MDL_FAC 1` so gz thrust is linear in PX4's command, and
   `MPC_THR_HOVER` at the hover collective. Saved SITL parameters from earlier runs are cleared
   by the WSL script, since PX4 only resets them when the autostart id changes.

## Flight (headless gz sim 8.15, PX4 v1.17.0, WSL2 Ubuntu 24.04)

`commander takeoff` from the PX4 console, then `commander land` 30 s later.

Two flights: the first with the ten rotor links adding 3.2 kg on top of the 12 kg body (hover
command mean 0.57, altitude std 1.4 cm, roll and pitch std 3.2 and 3.8 deg), then the final one
below with the body mass reduced by the rotor masses so the model weighs the scenario's 12 kg.

| quantity | value |
|---|---|
| flight | 42 s armed: climb, 30 s hover, land, auto-disarm |
| climb to takeoff altitude | 2.5 m above start, max 1.04 m/s |
| altitude hold in hover | 2.4 cm standard deviation |
| attitude in hover | roll -0.1 +- 2.5 deg, pitch -0.1 +- 2.1 deg, yaw drift 11 deg |
| peak attitude over the flight | roll 5.9 deg, pitch 7.7 deg |
| position drift in hover | 0.39 m max |
| hover motor commands, rotors 0 to 9 | 0.43 0.43 0.49 0.49 0.46 0.46 0.49 0.49 0.39 0.35, mean 0.449 |
| tiltlab hover prediction for the same set | 0.43 0.43 0.49 0.49 0.46 0.46 0.49 0.49 0.39 0.34, mean 0.449 |
| touchdown speed | 0.0 m/s, auto-disarm |

PX4's allocator in Gazebo settled on the same per-motor hover distribution tiltlab's port of the
allocator predicts, to two decimals on every motor.

## Reproduce

```bash
wsl -d Ubuntu-24.04 bash "/mnt/c/Users/stefa/Documents/Utopia Labs/wsl/tiltlab_gazebo.sh" \
  --mode sitl --harness "/mnt/c/Users/stefa/Documents/Utopia Labs/vibe-coded/exports/gazebo/atlas_phase01_cad_hover"
```

Wait for `Ready for takeoff!`, give the height estimate another ten seconds, then in the PX4
console `commander takeoff` and later `commander land`. The gz motor speeds and the vehicle can
be watched with `gz topic -e -t /atlas_phase01_cad_hover_0/command/motor_speed` and the Gazebo GUI.
