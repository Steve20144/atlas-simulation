# Golden fixtures

Copied 2026-09-09 from `C:\Users\stefa\Documents\QGroundControl Daily\{Logs,Parameters}`. Flight controller: Pixhawk 6X, PX4 v1.17.0 (git d6f12ad1c4), airframe SYS_AUTOSTART 4001 with CA_ROTOR_COUNT 10, CA_AIRFRAME 0, CA_METHOD 2, CA_R_REV 0, MC_AIRMODE 0, THR_MDL_FAC 0, MC_YAW_TQ_CUTOFF 2, MPC_THR_HOVER 0.5, MPC_MANTHR_MIN 0.08 in every log. The vehicle was on the rig for all logs (roll and position locked, pitch and yaw free).

Rule: golden tests read parameters from the log (`ULog.initial_parameters`), not from the `.params` files. Example: `14150909.params` (saved 14:16) has CA_ROTOR0_KM = -0.05, but log 36 (ended 14:15) ran with CA_ROTOR0_KM = 0.

`ulog_survey.json` next to this file holds the full topic list, field names, parameter snapshot and the numbers below, produced by pyulog.

## Logging rates (all three logs)
| topic | rate |
|---|---|
| vehicle_torque_setpoint, vehicle_thrust_setpoint | 50 Hz |
| actuator_motors | 10 Hz |
| control_allocator_status | 5 Hz |
| vehicle_rates_setpoint, vehicle_angular_velocity, manual_control_setpoint, rate_ctrl_status | see ulog_survey.json |

The allocator itself ran much faster than the logger. Compare replica output at the logged `actuator_motors` and `control_allocator_status` timestamps using the most recent preceding torque and thrust setpoint sample (zero-order hold), and report the residual separately from the tolerance check. `vehicle_thrust_setpoint.xyz[0]` and `xyz[1]` are exactly zero throughout every log (stock PX4 multicopter controllers command body-Z thrust only).

## log_36_2026-9-9-14-11-36.ulg (51.9 s): tilted-axis geometry, rear fans starved
- Rotors 0 to 7: AX = 1, AY = 0, AZ = -1 (PX4 normalises the axis), CT = 6.5, KM = 0. Rotors 8, 9: vertical, CT = 6.5, KM = 0. Positions as in the current parameter files.
- `control_allocator_status.unallocated_thrust` at 33 s = (0, 0, -0.9355). Minimum over the log = -0.961.
- `actuator_saturation[0..7]` = -2 (lower limit) on 100 percent of status samples; rotors 8 and 9 = 0.
- Over the 22 status samples with |yaw torque setpoint| > 0.05, unallocated yaw / commanded yaw = 1.00 (yaw entirely unallocated).
- Thrust setpoint z in [-1.0, 0].

## log_40_2026-9-9-15-09-54.ulg (62.7 s): vertical axes, KM-based yaw model, low collective on the rig
- Rotors 0 to 7: AX = AY = 0, AZ = -1, CT = 5.6, KM = -0.15/+0.15 (0/1), -0.13/+0.13 (2/3), -0.10/+0.10 (4/5), -0.08/+0.08 (6/7). Rotors 8, 9: vertical, CT = 6.5, KM = 0.
- Over the 69 status samples with |yaw torque setpoint| > 0.05 (linear interpolation of the setpoint to status time): mean(|unallocated yaw| / |yaw sp|) = 0.778, sum ratio = 0.788 (the "78 percent unallocated yaw" fact). With zero-order-hold pairing, as the allocator and the golden test use, the same selection gives 70 samples and 0.7896 for the logged data; the replica gives 0.7913.
- Thrust setpoint z in [-0.218, 0]. Unallocated thrust z stays within [-3e-8, 0.021].
- Rotors 0 to 7 all saturated low on only 2.9 percent of status samples (individual rotors saturate low far more often).
- Attitude on the rig from vehicle_attitude: roll median -2.25 deg (5th to 95th percentile -5.5 to -1.2), pitch median +6.27 deg (5th to 95th percentile -3.2 to +6.9).
- Flight mode: vehicle_status.nav_state = 15 (Stabilized) for the whole log; arming_state goes 1 (standby) to 2 (armed); vehicle_land_detected.landed takes both 0 and 1.
- rate_ctrl_status integrators: pitchspeed_integ in [-0.300, 0.000] (pinned at the MC_PR_INT_LIM floor while the rig held pitch), rollspeed_integ in [-0.005, 0.086], yawspeed_integ in [-0.005, 0.001].
- manual_control_setpoint (fields roll, pitch, yaw, throttle, ...) is logged at only 5 Hz. Replay tests (M7) must hold the last stick sample (zero-order hold) and state that this limits achievable agreement.

## log_27_2026-9-9-13-33-50.ulg (6.3 s): short spool-up with tilted geometry
- Same tilted geometry as log 36 but CA_ROTOR0_KM = -0.05. Thrust setpoint z in [-0.219, 0]. Used for the landed-throttle-floor and spool-up behaviour of the controller replica (M7).

## log_26_2026-9-9-13-14-02.ulg (short)
- Earlier rig session, kept for reference only.

## Parameter files
- `14150909.params`: tilted geometry (AX = 1) as flown in log 36, saved after the flight (KM of rotor 0 changed to -0.05).
- `params_v3_dihedral_yaw.params`, `20260909_1515_params_v4_rig_airmode.params`: vertical axes with the KM yaw model as flown in log 40 (v4 adds MC_AIRMODE = 2).
- `working09091439.params`, `param090926.params`: intermediate backups.
- Format: `# comment` header lines, then tab-separated `vehicle_id component_id name value type`, CRLF line endings. Type codes: 6 = INT32, 9 = FLOAT.

## PX4 status message semantics (verify against msg/ControlAllocatorStatus.msg)
- `actuator_saturation[i]`: 0 = OK, -2 = lower limit, 2 = upper limit (dynamic variants -1 / 1).
- `unallocated_torque[3]`, `unallocated_thrust[3]`: requested minus achieved, in normalised control units.
