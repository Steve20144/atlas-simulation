# PX4 v1.17.0 control allocation: Python port specification

Derived only from the pinned tree `third_party/PX4-Autopilot` (tag v1.17.0, commit
d6f12ad1c4f70ad3230afd7d86e971421e02fef4); every statement cites `file:lines` and every formula is quoted.

File tags (paths relative to `third_party/PX4-Autopilot`):

| Tag | Path |
|---|---|
| AER | `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp` (`.hpp` = AER.hpp) |
| AEM | `.../VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.cpp` (`.hpp` = AEM.hpp) |
| AE | `src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.cpp` (`.hpp` = AE.hpp) |
| CA | `src/lib/control_allocation/control_allocation/ControlAllocation.cpp` (`.hpp` = CA.hpp) |
| CAPI | `.../control_allocation/ControlAllocationPseudoInverse.cpp` (`.hpp` = CAPI.hpp) |
| CASD | `.../control_allocation/ControlAllocationSequentialDesaturation.cpp` (`.hpp` = CASD.hpp) |
| PINV / SQM / MAT / SLICE / VEC3 | `src/lib/matrix/matrix/{PseudoInverse,SquareMatrix,Matrix,Slice,Vector3}.hpp` |
| MOD | `src/modules/control_allocator/ControlAllocator.cpp` (`.hpp` = MOD.hpp) |
| YAML | `src/modules/control_allocator/module.yaml` |
| FM | `src/lib/mixer_module/functions/FunctionMotors.hpp` |
| MRC | `src/modules/mc_rate_control/MulticopterRateControl.cpp` |

Sizes: `NUM_ACTUATORS = 16`, `NUM_AXES = 6`, axis order `ROLL, PITCH, YAW, THRUST_X, THRUST_Y,
THRUST_Z` (AE.hpp:75-85). `EffectivenessMatrix` is `Matrix<float, 6, 16>`, `ActuatorVector` is
`Vector<float, 16>` (AE.hpp:89-90). All arithmetic is float32. `matrix::Matrix` storage is
zero-initialised (`Type _data[M][N] {};`, MAT:24), so unused rows, columns and actuator slots are 0.

## 1. Geometry from parameters (CA_AIRFRAME = 0)

### 1.1 Object construction and flags

`CA_AIRFRAME` 0 (Multirotor) and -1 (NONE) both instantiate `ActuatorEffectivenessMultirotor`
(MOD:214-218). It owns `ActuatorEffectivenessRotors _mc_rotors` built as `_mc_rotors(this)`
(AEM:38-42), i.e. defaults `AxisConfiguration::Configurable`, `tilt_support = false` (AER.hpp:82-83).
AEM.cpp:34-56 contains only the constructor and `getEffectivenessMatrix`; it never calls
`enablePropellerTorque`, `enableYawByDifferentialThrust`, `enablePropellerTorqueNonUpwards` or
`enableThreeDimensionalThrust` (callers: Custom:52, FixedWing:53, MCTilt:54, StandardVTOL:53,
Tailsitter:60 only), so all four `Geometry` flags keep their default `{false}` (AER.hpp:73-80):
`km` is used as configured, the yaw row is kept, thrust is 3D.

### 1.2 Parameter read (`updateParams`, AER:85-127)

`NUM_ROTORS_MAX = 12` (AER.hpp:63). Names `CA_ROTOR%u_PX/PY/PZ/AX/AY/AZ/CT/KM` (AER:52-75). YAML
defaults: PX/PY/PZ 0.0 (138,149,160), AX 0.0, AY 0.0, AZ -1.0 (172,183,194), CT 6.5 (209), KM 0.05
(225), CA_ROTOR_COUNT 0 (127).

```cpp
	_geometry.num_rotors = math::min(NUM_ROTORS_MAX, static_cast<int>(_param_ca_rotor_count.get()));   // AER:89
```

For `i < num_rotors`: position = (PX, PY, PZ) (AER:92-95); axis = (AX, AY, AZ) because the
configuration is `Configurable` (AER:99-104); `thrust_coef = CT`, `moment_ratio = KM` (AER:115-116);
`tilt_index = -1` (AER:123-125). Rotors `>= num_rotors` are never read and stay zero.

### 1.3 Matrix build (`addActuators`, `computeEffectivenessMatrix`, AER:129-225)

`getEffectivenessMatrix` returns `false` for `NO_EXTERNAL_UPDATE` (AEM:48-50), else calls
`_mc_rotors.addActuators(configuration)` (AEM:53), which fills
`configuration.effectiveness_matrices[selected_matrix]` from column 0 and registers the count as
MOTORS via `actuatorsAdded` (AER:137-140; AE:64-74). `Configuration::addActuator` (AE:38-62, one
column from a torque and a thrust vector) is not on the rotor path. `config.trim[]` and
`config.linearization_point[]` are never written for the multirotor; `Configuration config{}` is
value-initialised (MOD:466), so both are zero.

Per rotor (AER:150-222):

```cpp
		if (i + actuator_start_index >= NUM_ACTUATORS) { break; }
		++num_actuators;
		Vector3f axis = geometry.rotors[i].axis;
		float axis_norm = axis.norm();
		if (axis_norm > FLT_EPSILON) { axis /= axis_norm; } else { continue; }   // Bad axis definition, ignore this rotor
		const Vector3f &position = geometry.rotors[i].position;
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;
		if (geometry.propeller_torque_disabled) { km = 0.f; }
		if (geometry.propeller_torque_disabled_non_upwards) {   // AER:183-189
			bool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;   if (!upwards) { km = 0.f; }
		}
		if (fabsf(ct) < FLT_EPSILON) { continue; }
		matrix::Vector3f thrust = ct * axis;                                        // AER:196
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;      // AER:199
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
```

- `num_actuators` is incremented before the axis and CT checks (AER:156): a rotor with zero axis or
  `|CT| < FLT_EPSILON` still occupies an all-zero column and counts as a motor.
- `axis.norm()` is `sqrt(a.dot(a))` (`Vector.hpp:105-109`); the test is on the unnormalised norm.
- Cross product (VEC3:52) with `a = position`, `b = axis`:
  `{a(1)*b(2) - a(2)*b(1), -a(0)*b(2) + a(2)*b(0), a(0)*b(1) - a(1)*b(0)}`.
- KM sign: `- ct * km * axis`. For the default axis `(0, 0, -1)` the yaw row is `+ct*km` and the
  THRUST_Z row is `-ct`. YAML:217-218: positive KM for CCW rotors. Unit test
  `ActuatorEffectivenessRotorsTest.cpp:53,78,81`: KM 0.05, CT 1 gives yaw row `0.05`, thrust_z `-1.0`.
- The `propeller_torque_disabled*` branches (AER:179-189), yaw zeroing (AER:207-210) and 1D-thrust
  case (AER:212-221) are never active for CA_AIRFRAME = 0 (section 1.1).

### 1.4 Weak-row zeroing in the module (MOD:565-588)

```cpp
			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;
				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) { all_entries_small = false; }
				}
				if (all_entries_small) { matrix.row(n) = 0.f; }
			}
			...
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
```

A row survives only if some entry has `|entry| > 0.05` (strict). Planar multirotor THRUST_X/Y rows
are zero anyway; tilted fans with small in-plane components can lose a row here. Failed-motor columns
are zeroed first when `_handled_motor_failure_bitmask != 0` (MOD:539-558; needs CA_FAILURE_MODE > 0).

### 1.5 Actuator min, max, slew (MOD:479-563)

```cpp
						if (_param_r_rev.get() & (1u << actuator_type_idx)) { minimum[selected_matrix](...) = -1.f; }   // MOD:508-513
						else { minimum[selected_matrix](...) = 0.f; }
						slew_rate[selected_matrix](...) = _params.slew_rate_motors[actuator_type_idx];                    // MOD:515
					maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;                                // MOD:532
```

Motor i: `min = -1` if bit i of `CA_R_REV` (YAML:52-71, default 0) else `0`; `max = 1`; slew from
`CA_R{i}_SLEW` (YAML:72-87, seconds, default 0). `minimum`/`maximum` are default-constructed
(MOD:480-481), so slots `>= num_actuators` have `min = max = 0` (pitfall 5.10). Pushed via
`setActuatorMin/Max/SlewRateLimit` (MOD:561-563), overriding the constructor defaults (CA:44-49).

### 1.6 Linearisation point and control trim (`ControlAllocation::setEffectivenessMatrix`, CA:51-64)

```cpp
	_effectiveness = effectiveness;
	ActuatorVector linearization_point_clipped = linearization_point;
	clipActuatorSetpoint(linearization_point_clipped);
	_actuator_trim = actuator_trim + linearization_point_clipped;
	clipActuatorSetpoint(_actuator_trim);
	_num_actuators = num_actuators;
	_control_trim = _effectiveness * linearization_point_clipped;
```

Multirotor: `_actuator_trim = 0`, `_control_trim = 0`. `clipActuatorSetpoint` (CA:77-91) loops
`i < _num_actuators`, replaces by `_actuator_trim(i)` if `max < min`, else clamps to `[min, max]`.
`ControlAllocationPseudoInverse::setEffectivenessMatrix` (CAPI:44-59) calls the base, then sets
`_mix_update_needed = true`, `_normalization_needs_update = update_normalization_scale`
(`_metric_allocation` is never set for the multirotor).

## 2. Pseudo-inverse

### 2.1 `updatePseudoInverse` (CAPI:61-78)

```cpp
	if (_mix_update_needed) {
		matrix::geninv(_effectiveness, _mix);
		if (!_metric_allocation) {
			if (_normalization_needs_update && !_had_actuator_failure) {
				updateControlAllocationMatrixScale();
				_normalization_needs_update = false;
			}
			normalizeControlAllocationMatrix();
		}
		_mix_update_needed = false;
	}
```

`_mix` is `Matrix<float, 16, 6>` (CAPI.hpp:63): `_mix(i, axis)`, actuators along rows. The `geninv`
return value is ignored. The scale is recomputed only for `reason == CONFIGURATION_UPDATE` (MOD:588),
i.e. from `parameters_updated` (MOD:133), never from the 100 ms rate-limited `NO_EXTERNAL_UPDATE`
path (MOD:402, 468-471; AEM:48-50). After a handled motor failure the mix is recomputed but the scale
is frozen (CA.hpp:108, MOD:741-743).

### 2.2 `geninv` (PINV:24-63) and `fullRankCholesky` (PINV:78-126)

`G` is 6x16, `M = 6 <= N = 16`, first branch (PINV:29-43):

```cpp
		SquareMatrix<Type, M> A = G * G.transpose();
		SquareMatrix<Type, M> L = fullRankCholesky(A, rank);
		A = L.transpose() * L;
		SquareMatrix<Type, M> X;
		if (!inv(A, X, rank)) { res = Matrix<Type, N, M>(); return false; }
		A = X * X * L.transpose();
		res = G.transpose() * (L * A);
```

i.e. `G+ = G^T L (L^T L)^-1 (L^T L)^-1 L^T` (Courrieu 2008, PINV:22). `fullRankCholesky`:

```cpp
	const Type tol = N * typeEpsilon<Type>() * A.diag().max();          // N = 6 here, typeEpsilon<float> = FLT_EPSILON (PINV:69-73)
	Matrix<Type, N, N> L;
	size_t r = 0;
	for (size_t k = 0; k < N; k++) {
		if (r == 0) { for (size_t i = k; i < N; i++) { L(i, r) = A(i, k); } }
		else {
			for (size_t i = k; i < N; i++) {
				Type LL = Type();
				for (size_t j = 0; j < r; j++) { LL += L(i, j) * L(k, j); }
				L(i, r) = A(i, k) - LL;
			}
		}
		if (L(k, r) > tol) {
			L(k, r) = std::sqrt(L(k, r));
			if (k < N - 1) { for (size_t i = k + 1; i < N; i++) { L(i, r) = L(i, r) / L(k, r); } }
			r = r + 1;
		}
	}
	rank = r;
```

`tol = 6 * 1.1920929e-7 * max_i (G G^T)(i,i)`: an absolute threshold on a squared quantity (the
diagonal of `G G^T` is the squared norm of each axis row). No pivoting: axes are tested in order roll,
pitch, yaw, tx, ty, tz. A rejected axis `k` leaves `L(k, r)` holding its residual (`<= tol`); rows
`> k` of that column are overwritten next iteration. `inv(A, X, rank)` (SQM:378-501) only processes
the leading `rank x rank` block (loops to `rank`, SQM:390-487) and leaves identity elsewhere
(SQM:384-385), so the leftover contributes `O(tol^2)` to `res`: negligible, not exactly zero. `inv` is
LU with a row swap only when `|U(n,n)| < FLT_EPSILON` (SQM:393-407), failure if still `< FLT_EPSILON`
(SQM:418-420), finiteness check (SQM:490-496). Products accumulate in float32 as
`for i, for k, for j: res(i,k) += self(i,j) * other(j,k)` (MAT:159-173).

Rank handling: exactly-zero axis rows (planar THRUST_X/Y, rows deleted by the 0.05 rule) give
`L(k, r) = 0`, are rejected, and `G+` has exactly zero columns for them.

`numpy.linalg.pinv` equivalence: the Moore-Penrose inverse is unique, so `pinv(G) == geninv(G)`
(to rounding) whenever both make the same rank decision, i.e. every axis row is either exactly zero
or clearly non-degenerate (our multirotor cases after the 0.05 rule). Not equivalent when a row is
non-zero but its Schur complement in `G G^T` is `<= tol`: PX4 drops it, `pinv` (default `rcond =
1e-15`, relative to the largest singular value) keeps it and yields huge entries. Recommendation:
port `geninv` and `fullRankCholesky` line by line with `tol` from `np.finfo(np.float32).eps`; use
`pinv` only as a cross-check.

### 2.3 `updateControlAllocationMatrixScale` (CAPI:80-148)

`_normalize_rpy` comes from `ActuatorEffectivenessMultirotor::getNormalizeRPY`, `normalize[0] =
true` (AEM.hpp:52-55; same in AER.hpp:93-96), read at MOD:166-167 and applied with `setNormalizeRPY`
(MOD:195). So the `if (_normalize_rpy)` branch runs.

```cpp
		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;
		for (int i = 0; i < _num_actuators; i++) {
			if (fabsf(_mix(i, 0)) > 1e-3f) { ++num_non_zero_roll_torque; }
			if (fabsf(_mix(i, 1)) > 1e-3f) { ++num_non_zero_pitch_torque; }
		}
		float roll_norm_scale = 1.f;
		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
		}
		float pitch_norm_scale = 1.f;   // same formula with _mix.col(1) and num_non_zero_pitch_torque (CAPI:106-110)
		_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
		_control_allocation_scale(1) = _control_allocation_scale(0);
		// Scale yaw separately
		_control_allocation_scale(2) = _mix.col(2).max();
	...
	_control_allocation_scale(THRUST_Z) = 1.f;
	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;   float norm_sum = 0.f;
		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(_mix(i, 3 + axis_idx));
			norm_sum += norm;
			if (norm > FLT_EPSILON) { ++num_non_zero_thrust; }
		}
		if (num_non_zero_thrust > 0) { _control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust; }
		else { _control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z); }
	}
```

- Counts use `> 1e-3f` on the raw pre-normalisation mix, rows `< _num_actuators` only.
  `_mix.col(0).norm_squared()` sums all 16 rows (SLICE:319-331); unused rows are 0. `count / 2.f` is
  float division.
- `_mix.col(2).max()` is the SIGNED max over all 16 rows starting from row 0 (SLICE:343-358); with
  `_num_actuators < 16` it is `max(0, max over used rows)`. Not an absolute value.
- Thrust order z, then y, then x: the z scale is the mean of the non-zero absolute entries of column
  5 (test `> FLT_EPSILON`); y and x with no non-zero entries copy the z scale (CAPI:124-125).

### 2.4 `normalizeControlAllocationMatrix` (CAPI:150-177)

```cpp
	if (_control_allocation_scale(0) > FLT_EPSILON) { _mix.col(0) /= _control_allocation_scale(0);   _mix.col(1) /= _control_allocation_scale(1); }
	if (_control_allocation_scale(2) > FLT_EPSILON) { _mix.col(2) /= _control_allocation_scale(2); }
	if (_control_allocation_scale(3) > FLT_EPSILON) {
		_mix.col(3) /= _control_allocation_scale(3);   _mix.col(4) /= _control_allocation_scale(4);   _mix.col(5) /= _control_allocation_scale(5);
	}
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) { if (fabsf(_mix(i, j)) < 1e-3f) { _mix(i, j) = 0.f; } }
	}
```

Columns 0, 1 gated on scale(0); columns 3, 4, 5 gated on scale(3) (the x scale, equal to the z scale
for a planar vehicle by the copy rule). Yaw stays unscaled if its max is `<= FLT_EPSILON`. This runs on
every `_mix_update_needed` with the stored scale; the 1e-3 zeroing acts on normalised entries.

### 2.5 `allocate()` for CA_METHOD = 0 (CAPI:179-189)

```cpp
	updatePseudoInverse();
	_prev_actuator_sp = _actuator_sp;
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
```

No desaturation; clipping happens in the module (section 4.3).

## 3. Sequential desaturation (CA_METHOD = 1, or AUTO for the multirotor)

The class derives from `ControlAllocationPseudoInverse` and `ModuleParams` (CASD.hpp:50) and owns
`(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode` (CASD.hpp:129-131). `MC_AIRMODE` is defined
in `src/lib/mixer_module/params.c:18`, default 0 (0 Disabled, 1 Roll/Pitch, 2 Roll/Pitch/Yaw,
lines 13-15), refreshed by `updateParameters()` -> `updateParams()` (CASD:230-234) from MOD:129-131.
`MINIMUM_YAW_MARGIN = 0.15f` (CASD.hpp:63).

### 3.1 `allocate()` (CASD:44-65)

```cpp
	updatePseudoInverse();
	_prev_actuator_sp = _actuator_sp;
	switch (_param_mc_airmode.get()) {
	case 1:  mixAirmodeRP();       break;
	case 2:  mixAirmodeRPY();      break;
	default: mixAirmodeDisabled(); break;
	}
```

### 3.2 `computeDesaturationGain` (CASD:88-119)

```cpp
	float k_min = 0.f;
	float k_max = 0.f;
	for (int i = 0; i < _num_actuators; i++) {
		if (fabsf(desaturation_vector(i)) < 0.2f) { continue; }   // weak effectiveness: skip
		if (actuator_sp(i) < _actuator_min(i)) {
			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);
			if (k < k_min) { k_min = k; }   if (k > k_max) { k_max = k; }
		}
		if (actuator_sp(i) > _actuator_max(i)) {
			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);
			if (k < k_min) { k_min = k; }   if (k > k_max) { k_max = k; }
		}
	}
	return k_min + k_max;
```

The 0.2 threshold is on the NORMALISED mix column (section 2.4), so the scale decides which motors
participate. `k_min <= 0 <= k_max`; the gain is their sum.

### 3.3 `desaturateActuators` (CASD:67-86)

```cpp
	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);
	if (increase_only && gain < 0.f) { return; }
	for (int i = 0; i < _num_actuators; i++) { actuator_sp(i) += gain * desaturation_vector(i); }
	gain = 0.5f * computeDesaturationGain(desaturation_vector, actuator_sp);
	for (int i = 0; i < _num_actuators; i++) { actuator_sp(i) += gain * desaturation_vector(i); }
```

Two passes: full gain, then half of the recomputed gain (applied regardless of sign). For upward
rotors the THRUST_Z effectiveness is `-ct` (AER:196, 204), so `_mix(i, THRUST_Z) < 0` and a positive
gain along `thrust_z` LOWERS motor outputs: `increase_only` along `thrust_z` means "only reduce
collective thrust" (comments CASD:176, 195, 226).

### 3.4 The three modes and `mixYaw`

Every mode first writes, for `i < _num_actuators`, `_actuator_sp(i) = _actuator_trim(i) + sum
_mix(i, axis) * (_control_sp(axis) - _control_trim(axis))` with `=` (CASD:130, 155, 184), so nothing
survives from the previous iteration except `_prev_actuator_sp` (slew). THRUST_X and THRUST_Y are
always in this first step (CASD:133-134, 159-160, 187-188) and are never desaturation directions.

- `mixAirmodeRP` (CASD:121-143): mix ROLL, PITCH, THRUST_X/Y/Z (no yaw);
  `desaturateActuators(_actuator_sp, thrust_z)` both directions; `mixYaw()`.
- `mixAirmodeRPY` (CASD:145-171): mix all six axes; `desaturateActuators(_actuator_sp, thrust_z)`;
  `desaturateActuators(_actuator_sp, yaw)` ("prioritize roll/pitch over yaw", CASD:168-169). No `mixYaw`.
- `mixAirmodeDisabled` (CASD:173-204): mix without yaw, then

```cpp
	desaturateActuators(_actuator_sp, thrust_z, true);   // only reduce thrust
	desaturateActuators(_actuator_sp, roll);             // Reduce roll/pitch acceleration if needed to unsaturate
	desaturateActuators(_actuator_sp, pitch);
	mixYaw();
```

`mixYaw` (CASD:206-228):

```cpp
	for (int i = 0; i < _num_actuators; i++) {
		_actuator_sp(i) += _mix(i, ControlAxis::YAW) * (_control_sp(ControlAxis::YAW) - _control_trim(ControlAxis::YAW));
		yaw(i) = _mix(i, ControlAxis::YAW);   thrust_z(i) = _mix(i, ControlAxis::THRUST_Z);
	}
	ActuatorVector max_prev = _actuator_max;
	_actuator_max += (_actuator_max - _actuator_min) * MINIMUM_YAW_MARGIN;
	desaturateActuators(_actuator_sp, yaw);
	_actuator_max = max_prev;
	desaturateActuators(_actuator_sp, thrust_z, true);   // reduce thrust only
```

The upper limit becomes `max + 0.15 * (max - min)` (1.15 for a non-reversible motor) for the yaw pass
only, then is restored; the final thrust-only pass pulls outputs above 1.0 down by reducing thrust.

### 3.5 MC_YAW_TQ_CUTOFF

Does not appear under `src/lib/control_allocation` or `src/modules/control_allocator`. It is used only
in `mc_rate_control`: `_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get())` (MRC:99) and
`torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt)` (MRC:223), before
`vehicle_torque_setpoint` is published (MRC:236-238, 264). The logged torque already contains the
filter and the battery scaling (MRC:252-253); the port must apply neither.

## 4. ControlAllocator module glue (MOD)

### 4.1 Control setpoint vector (MOD:376-411)

```cpp
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);                       // MOD:378
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);   do_update = true;               // MOD:386-388
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;
	}
	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) { _thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz); }
	if (do_update) {
		c[0](0) = _torque_sp(0); c[0](1) = _torque_sp(1); c[0](2) = _torque_sp(2);                     // MOD:406-411
		c[0](3) = _thrust_sp(0); c[0](4) = _thrust_sp(1); c[0](5) = _thrust_sp(2);
```

Allocation runs on each new `vehicle_torque_setpoint` (callback topic, MOD.hpp:178) with the latest
`vehicle_thrust_setpoint`; `mc_rate_control` publishes thrust then torque in one iteration (MRC:260,
264), so the pair with equal `timestamp_sample` is used. Torque xyz is normalised, thrust xyz in
[-1, 1] (`msg/VehicleTorqueSetpoint.msg:5`, `msg/VehicleThrustSetpoint.msg:5`); `_thrust_sp(2) < 0`
for upward thrust (MRC:166: `-(throttle + 1.f) * .5f`).

### 4.2 Method selection (`update_allocation_method`, MOD:136-202)

```cpp
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();              // MOD:139
			AllocationMethod method = configured_method;
			if (configured_method == AllocationMethod::AUTO) { method = desired_methods[i]; }  // MOD:172-174
			case AllocationMethod::PSEUDO_INVERSE:          _control_allocation[i] = new ControlAllocationPseudoInverse(); break;
			case AllocationMethod::SEQUENTIAL_DESATURATION: _control_allocation[i] = new ControlAllocationSequentialDesaturation(); break;
			_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);   _control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);   // MOD:195-196
```

Enum `PSEUDO_INVERSE = 0, SEQUENTIAL_DESATURATION = 1, AUTO = 2` (AE.hpp:49-54); CA_METHOD default 2
(YAML:38-49). `desired_methods` comes from `getDesiredAllocationMethod` (MOD:163-164), which for the
multirotor returns `SEQUENTIAL_DESATURATION` (AEM.hpp:47-50), so AUTO behaves as method 1.
`numMatrices()` is 1 (AE.hpp:140). The previous setpoint is carried into the new object via
`setActuatorSetpoint` (clips, CA:66-75).

### 4.3 After `allocate()` (MOD:427-442)

```cpp
			_control_allocation[i]->setControlSetpoint(c[i]);   _control_allocation[i]->allocate();
			_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp);
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp, /* min, max */);
			if (_has_slew_rate) { _control_allocation[i]->applySlewRateLimit(dt); }
			_control_allocation[i]->clipActuatorSetpoint();
```

`allocateAuxilaryControls` and `updateSetpoint` are empty base virtuals (AE.hpp:190, 198-199) not
overridden by the multirotor: no-ops. `_has_slew_rate` is true if any `CA_R{i}_SLEW` or
`CA_SV{i}_SLEW` `> FLT_EPSILON` (MOD:108-118). Slew (CA:111-126):

```cpp
		if (_actuator_slew_rate_limit(i) > FLT_EPSILON) {
			float delta_sp_max = dt * (_actuator_max(i) - _actuator_min(i)) / _actuator_slew_rate_limit(i);
			float delta_sp = _actuator_sp(i) - _prev_actuator_sp(i);
			if (delta_sp > delta_sp_max) { _actuator_sp(i) = _prev_actuator_sp(i) + delta_sp_max; }
			else if (delta_sp < -delta_sp_max) { _actuator_sp(i) = _prev_actuator_sp(i) - delta_sp_max; }
		}
```

`_prev_actuator_sp` is the previous iteration's final clipped setpoint (captured at CAPI:185 /
CASD:50). `clipActuatorSetpoint()` then clamps to `[min, max]` (CA:77-91). Actuator min/max are only
set in `update_effectiveness_matrix_if_needed` (MOD:561-562), never in the run loop.

### 4.4 `actuator_motors` (`publish_actuator_controls`, MOD:649-693)

```cpp
	if (!_publish_controls) { return; }        // vehicle_control_mode.flag_control_allocation_enabled, MOD:371-373
	actuator_motors.reversible_flags = _param_r_rev.get();
	uint32_t stopped_motors = _actuator_effectiveness->getStoppedMotors() | _handled_motor_failure_bitmask | _motor_stop_mask;
	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;          // actuator_sp = clipped setpoint, MOD:678
		if (stopped_motors & (1u << motors_idx)) { actuator_motors.control[motors_idx] = NAN; }
	}
	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) { actuator_motors.control[i] = NAN; }
```

`NUM_CONTROLS = 12` (`msg/versioned/ActuatorMotors.msg:15`). `control[i]` is the clipped allocator
output, `[0, 1]` non-reversible, `[-1, 1]` reversible; `i >= CA_ROTOR_COUNT` is NaN. `getStoppedMotors()`
returns `_stopped_motors_mask` (AE.hpp:204), modified only by `stopMaskedMotorsWithZeroThrust`
(AE:87-103), which the multirotor never calls: 0.

THR_MDL_FAC is NOT applied in the control allocator. It belongs to the output driver
(`src/lib/mixer_module/motor_params.c:57`, default 0.0; model "rel_thrust = factor * rel_signal^2 +
(1-factor) * rel_signal", lines 47-49) and is applied to the already published `actuator_motors` in
`FunctionMotors::updateValues` (FM:81-120):

```cpp
		if (thrust_factor > FLT_EPSILON && thrust_factor <= 1.f) {
			const float a = thrust_factor;
			const float b = (1.f - thrust_factor);
			const float tmp1 = b / (2.f * a);
			const float tmp2 = b * b / (4.f * a * a);
			...	if (control > FLT_EPSILON)       { values[i] = -tmp1 + sqrtf(tmp2 + (control / a)); }
				else if (control < -FLT_EPSILON) { values[i] =  tmp1 - sqrtf(tmp2 - (control / a)); }
		}
		...	if ((reversible & (1u << i)) == 0) {
				if (values[i] < -FLT_EPSILON) { values[i] = NAN; }
				else { values[i] = values[i] * 2.f - 1.f; }      // remap from [0, 1] to [-1, 1]
			}
```

The logged `actuator_motors.control` is therefore linear allocator output; a replica compared to that
topic must not apply THR_MDL_FAC. The CT description "Thrust = CT * u^2" (YAML:200-202) is physical
meaning only; the allocator is linear in `u` (AER:196, 199).

### 4.5 `control_allocator_status` (`publish_control_allocator_status`, MOD:596-647)

Published at most every 5 ms (MOD:450-458), after the loop, from the CLIPPED setpoint.

```cpp
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() - allocated_control;
	control_allocator_status.unallocated_torque[0..2] = unallocated_control(0..2);   // MOD:610-612; unallocated_thrust[0..2] = unallocated_control(3..5), MOD:613-615
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);   // no-op for multirotor, AE.hpp:210
	control_allocator_status.torque_setpoint_achieved = (Vector3f(unallocated_torque).norm_squared() < 1e-6f);   // MOD:621-626, same for thrust
	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) { control_allocator_status.actuator_saturation[i] = ACTUATOR_SATURATION_UPPER; }
		else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) { control_allocator_status.actuator_saturation[i] = ACTUATOR_SATURATION_LOWER; }
	}
```

with `getAllocatedControl()` (CA.hpp:145-146):

```cpp
	{ return (_effectiveness * (_actuator_sp - _actuator_trim)).emult(_control_allocation_scale); }
```

So `unallocated = control_sp - scale .* (B * u_clipped)`, `B` the UNNORMALISED effectiveness matrix
(after the 0.05 row rule), `scale` the 6-vector of section 2.3; this puts unallocated values in the
setpoint's normalised units. Codes (`msg/ControlAllocatorStatus.msg:11-15`): OK 0, UPPER_DYN 1,
UPPER 2, LOWER_DYN -1, LOWER -2. The `_DYN` variants are never assigned in any `.c/.cpp/.hpp/.h`
under `src/` of this tree (grep confirmed): a replica produces only 0, 2, -2. No hysteresis: the
status is a pure function of the current `_actuator_sp`, `_actuator_min`, `_actuator_max`; the only
carried state is the 5 ms timer. The loop covers all 16 slots (pitfall 5.10).

## 5. Numerical pitfalls for a float64 numpy port (target: match float32 PX4 to about 1e-3)

1. Reproduce thresholds literally with `eps32 = np.finfo(np.float32).eps = 1.1920929e-7`: axis norm
   `> eps32` (AER:164); `|CT| < eps32` skip (AER:191); weak-row `> 0.05` strict (MOD:575); Cholesky
   `tol = 6 * eps32 * max(diag(B B^T))` (PINV:83); LU pivot `< eps32` (SQM:393, 418); mix counts
   `> 1e-3` (CAPI:91, 95); thrust count `> eps32` (CAPI:136); normalisation gates `> eps32`
   (CAPI:153, 158, 162); zeroing `< 1e-3` (CAPI:172); desaturation skip `< 0.2` (CASD:96);
   saturation `sp > max - eps32`, `sp < min + eps32` (MOD:634, 637); achieved `norm_squared < 1e-6`
   (MOD:623, 626).
2. The 1e-3 zeroing after normalisation is a discontinuity: an entry near 1e-3 may be zeroed in
   float32 but kept in float64. Run `geninv` and the normalisation in `np.float32` for an exact mix.
3. No integer division: `num_non_zero_roll_torque / 2.f` (CAPI:103) and `norm_sum /
   num_non_zero_thrust` (CAPI:142) are float divisions. Do not floor.
4. `_mix.col(2).max()` (CAPI:116) is a signed max over the full 16-row column: use
   `max(0.0, mix[:n, 2].max())` for `n < 16`, never `abs`.
5. Thrust scale order z, y, x (CAPI:128); y and x fall back to the z scale when they have no
   non-zero entries; columns 3-5 divided only if `scale[3] > eps32`, columns 0-1 only if `scale[0] > eps32`.
6. Mix and scale are computed once per parameter update, not per sample (section 2.1): build them
   once from the log's initial parameters. Float32 product accumulation order is `for i, for k, for j`
   (MAT:159-173); float64 differs at ~1e-7, fine for 1e-3, but use float32 in that order to bit-match.
8. Per-sample order: `_prev = sp`; mix; desaturate exactly as in section 3.4 (two passes each, 0.5 on
   the second, `increase_only` rejects only a negative FIRST gain, and its meaning depends on the
   negative sign of the thrust_z mix column: do not flip signs); `mixYaw` widens max by
   `0.15 * (max - min)` for the yaw pass only; slew if enabled; clip to `[min, max]`; publish motors;
   status from the clipped vector.
9. `k_min`, `k_max` start at 0 (CASD:91-92); the gain is `k_min + k_max`, not an extremum.
10. Unused slots `i >= CA_ROTOR_COUNT` have `min = max = 0`, `sp = 0` (MOD:480-481; MOD:532 sets only
    used slots; MAT:24 zero-init). `0 > 0 - eps32` is true, so MOD:634 marks them
    `ACTUATOR_SATURATION_UPPER = 2`. Reproduce this when comparing the full `actuator_saturation[16]`
    (verify on a fixture; open question 6.2).
11. `control_allocator_status` is published at most every 5 ms and only after a torque update, so it
    subsamples the actuator stream; compare by timestamp (nearest previous allocation), not by index.
    `actuator_motors.control` is pre THR_MDL_FAC (section 4.4) and can be negative only for
    reversible motors.
12. Slew dt is clamped to `[0.0002, 0.02]` s (MOD:378); with all `CA_R*_SLEW = 0` slew is skipped
    entirely (MOD:437). `CA_R_REV` bits change `(max - min)` to 2 in the yaw margin and slew formulas.
13. Cross product order (VEC3:52) and the `- ct * km * axis` sign (AER:199): with `axis = (0,0,-1)`
    yaw effectiveness is `+ct*km`, thrust_z effectiveness `-ct`. The weak-row rule (MOD:571-583) is
    applied BEFORE the pseudo-inverse; `getAllocatedControl` uses the same row-zeroed matrix.

## 6. Open questions not settled from the source

1. Pairing of `vehicle_thrust_setpoint` with each torque sample depends on uORB delivery order at run
   time (MOD:385-395); thrust is published first in the same rate-control iteration (MRC:260, 264) but
   nothing guarantees the allocator does not run between the two. Pair by equal `timestamp_sample`
   and check against the logs.
2. Whether logged `actuator_saturation[i]` for `i >= CA_ROTOR_COUNT` is indeed 2 (pitfall 5.10; MOD:634 is unconditional). Verify on a fixture.
3. `_had_actuator_failure` freezes the normalisation scale after a handled motor failure (CAPI:68);
   if a fixture ran with CA_FAILURE_MODE > 0 and a failure, the pre-failure scale must be rebuilt
   from the pre-failure matrix.
4. The rejected residual left in `L(k, rank)` by `fullRankCholesky` (section 2.2) has an `O(tol^2)` effect not evaluated for a real 10-fan geometry; expected far below 1e-3 but unverified.
5. `parameters_updated` re-runs on any parameter change (MOD.hpp:191) and rebuilds mix and scale
   mid-flight; replaying from `initial_parameters` is correct only if no CA_* or MC_AIRMODE
   parameter changed during the log.
6. `flag_control_allocation_enabled` gates `actuator_motors` publication (MOD:371-373, 652-654) but
   not the status; intervals with no motor output must be taken from the log.
