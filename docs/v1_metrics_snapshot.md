# v1 metrics snapshot (lean first version)

Straight from `POST /api/metrics` (uvicorn on :8000, frontend build served at `/`), run on 2026-09-09 against `scenarios/*.json` at the collective that solves hover (`collective` omitted). CT comes from the fan curve at cmd 1.0 (33.3 N per fan), not from the logged CA_ROTORn_CT overrides; see the notes column. Both concepts share the same LP constraints, so per-axis authority is identical across concepts; the concepts differ in controlled axes, hover solver, allocator method and badges.

## Estimated inputs

Every row below is computed from **estimated** inputs. `estimated` is true for all four cases and `estimated_sources` lists: baseline_dihedral30/stock: mass, fan_curve:xfly80_3280; flown_log40_vertical_km/stock: mass, fan_curve:xfly80_3280; flown_log36_ax1/stock: mass, fan_curve:xfly80_3280; baseline_dihedral30/fully_actuated: mass, fan_curve:xfly80_3280. Mass is a placeholder (no aircraft CAD on this machine) and the fan curve is a two-point linear placeholder (0 N / 0 W at cmd 0, 33.3 N / 2450 W at cmd 1), so absolute N, N m and W values are relative indicators only. Ratios between axes and between scenarios are the meaningful outputs.

## Table

| metric | baseline_dihedral30<br>stock | flown_log40_vertical_km<br>stock | flown_log36_ax1<br>stock | baseline_dihedral30<br>fully_actuated |
|---|---|---|---|---|
| controlled axes | roll, pitch, yaw, Fz | roll, pitch, yaw, Fz | roll, pitch, yaw, Fz | roll, pitch, yaw, Fx, Fy, Fz |
| collective (fraction of Fz_max) | 0.396 | 0.353 | 0.462 | 0.396 |
| Fz_max_N (all fans u = 1) | 297.3 | 333.0 | 255.0 | 297.3 |
| weight_N | 117.7 | 117.7 | 117.7 | 117.7 |
| hover method | px4_allocator_CA_METHOD_2 | px4_allocator_CA_METHOD_2 | px4_allocator_CA_METHOD_2 | bounded_min_norm_least_squares |
| hover exact | True | True | False | True |
| hover power_W | 9581 | 8658 | 0 | 9581 |
| hover headroom (1 - max u) | 0.439 | 0.436 | 1.000 | 0.439 |
| hover max u | 0.561 | 0.564 | 0.000 | 0.561 |
| hover badge | stock_px4 | stock_px4 | stock_px4 | needs_fully_actuated_controller |
| authority roll | +1.14 / -1.14 N m | +3.84 / -3.84 N m | +unattainable / -unattainable N m | +1.14 / -1.14 N m |
| badge roll | stock_px4 | stock_px4 | stock_px4 | stock_px4 |
| authority pitch | +25.46 / -32.76 N m | +25.46 / -33.26 N m | +unattainable / -unattainable N m | +25.46 / -32.76 N m |
| badge pitch | stock_px4 | stock_px4 | stock_px4 | stock_px4 |
| authority yaw | +0.27 / -0.27 N m | +1.70 / -1.70 N m | +unattainable / -unattainable N m | +0.27 / -0.27 N m |
| badge yaw | stock_px4 | stock_px4 | stock_px4 | stock_px4 |
| authority Fx | n/a (not controlled) | n/a (not controlled) | n/a (not controlled) | +unattainable / -unattainable N |
| badge Fx | - | - | - | needs_fully_actuated_controller |
| authority Fy | n/a (not controlled) | n/a (not controlled) | n/a (not controlled) | +2.26 / -2.26 N |
| badge Fy | - | - | - | needs_fully_actuated_controller |
| authority Fz | +117.68 / -113.77 N | +117.68 / -119.26 N | +117.68 / -unattainable N | +117.68 / -113.77 N |
| badge Fz | stock_px4 | stock_px4 | stock_px4 | stock_px4 |
| min-norm yaw cmd per 0.5 N m | 0.937 | 0.173 | 0.007 | 0.937 |
| coupling max_offaxis_fraction | 0.001 | 0.000 | 1.000 | 0.040 |
| coupling allocator | CA_METHOD_2 | CA_METHOD_2 | CA_METHOD_2 | CA_METHOD_0 |
| conditioning rank / null space dim | 4 / 6 | 4 / 6 | 3 / 7 | 5 / 5 |
| condition number | 9.8 | 14.1 | - | - |
| singular values | 2.92, 2.84, 1.62, 0.30 | 3.21, 2.97, 1.66, 0.23 | 2.80, 2.46, 1.70, 0.00 | 4.02, 2.84, 1.62, 0.70, 0.04, 0.00 |
| score value | 0.431 | 0.441 | 0.400 | 0.403 |
| estimated | True | True | True | True |
| compute_ms | 16.4 | 14.0 | 10.9 | 18.8 |

## Notes returned by the API

- baseline_dihedral30 / stock: none
- flown_log40_vertical_km / stock: 10 CA_ROTORn_CT override(s) ignored: metrics use the fan curve CT [33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3] N
- flown_log36_ax1 / stock: 10 CA_ROTORn_CT override(s) ignored: metrics use the fan curve CT [33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3, 33.3] N; trim setpoint not exactly achievable at this collective (clipped); roll is unattainable at this collective (LP infeasible); pitch is unattainable at this collective (LP infeasible); yaw is unattainable at this collective (LP infeasible); at least one controlled axis has no authority; score is not comparable
- baseline_dihedral30 / fully_actuated: Fx is unattainable at this collective (LP infeasible); at least one controlled axis has no authority; score is not comparable

## Reading the table

- Badges: `stock_px4` means the axis or metric is reachable with the stock PX4 v1.17.0 multirotor stack (roll, pitch, yaw, Fz); `needs_fully_actuated_controller` marks Fx, Fy and any hover or coupling result that assumes a 6-axis allocator (CA_METHOD 0 with FD_FAIL_P/R = 0).
- Authority is the largest attainable step on one axis at the hover trim with all other controlled axes held at zero (LP over 0 <= u <= 1). `(unattainable)` marks an LP that was infeasible.
- The fully actuated hover solves all six rows; the stock hover goes through the ControlAllocatorReplica with the scenario CA_METHOD, so a stock hover that is not exact means the allocator could not zero the residual torque or thrust.
