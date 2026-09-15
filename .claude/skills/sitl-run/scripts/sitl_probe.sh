#!/usr/bin/env bash
# sitl_probe.sh: one snapshot of the running PX4 SITL, for "it armed but does not climb" and
# similar questions. Runs INSIDE WSL:
#
#   wsl -d Ubuntu-24.04 -- bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/sitl_probe.sh [PARAM ...]
#
# Prints the fields that matter from vehicle_status, vehicle_local_position, vehicle_attitude,
# vehicle_thrust_setpoint, vehicle_torque_setpoint, control_allocator_status and actuator_motors,
# then the given parameters (default: geometry sanity, hover thrust and the rate gains).
set -uo pipefail
R="$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs"
cd "$R" || { echo "no SITL rootfs at $R"; exit 2; }
pgrep -f "bin/px4" >/dev/null || { echo "no PX4 SITL running"; exit 2; }
lst() { timeout 6 ../bin/px4-listener "$1" 1 2>&1 | grep -E "$2" | head -"${3:-12}" | sed 's/^ */  /'; }
echo "== vehicle_status";          lst vehicle_status 'arming_state:|nav_state:|failsafe:|hil_state:|pre_flight_checks_pass'
echo "== vehicle_local_position";  lst vehicle_local_position '^ *z:|^ *vz:|dist_bottom:|xy_valid|z_valid'
echo "== vehicle_attitude q";      lst vehicle_attitude 'q\[' 4
echo "== vehicle_thrust_setpoint"; lst vehicle_thrust_setpoint 'xyz' 3
echo "== vehicle_torque_setpoint"; lst vehicle_torque_setpoint 'xyz' 3
echo "== control_allocator_status"; lst control_allocator_status 'torque_setpoint_achieved|thrust_setpoint_achieved|unallocated|actuator_saturation' 20
echo "== actuator_motors";         lst actuator_motors 'control\[' 12
echo "== params"
if [ $# -eq 0 ]; then
  set -- CA_ROTOR_COUNT CA_ROTOR0_CT CA_ROTOR0_PY CA_ROTOR8_AY CA_ROTOR9_AY SENS_BOARD_Y_OFF MPC_THR_HOVER THR_MDL_FAC \
         MC_ROLLRATE_P MC_PITCHRATE_P MC_YAWRATE_P MC_ROLL_P MC_PITCH_P MC_AIRMODE CA_METHOD MIS_TAKEOFF_ALT
fi
for p in "$@"; do timeout 5 ../bin/px4-param show -q "$p" 2>/dev/null | tail -1 | sed 's/^ */  /'; done
