"""Replica of the PX4 v1.17.0 control allocator (multirotor path, CA_AIRFRAME 0).

Ported from the pinned tree third_party/PX4-Autopilot (tag v1.17.0, d6f12ad1c4). File tags:
  CA   src/lib/control_allocation/control_allocation/ControlAllocation.cpp (.hpp)
  CAPI src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp (.hpp)
  CASD src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp
  PINV src/lib/matrix/matrix/PseudoInverse.hpp
  SQM  src/lib/matrix/matrix/SquareMatrix.hpp
  MAT  src/lib/matrix/matrix/Matrix.hpp
  MOD  src/modules/control_allocator/ControlAllocator.cpp
  FM   src/lib/mixer_module/functions/FunctionMotors.hpp

Units: the control setpoint is [torque_xyz (normalised, FRD body axes), thrust_xyz (normalised,
FRD body axes, -1..1, negative z is upward thrust)]; actuator setpoints are normalised motor
commands in [min, max] = [0, 1] (or [-1, 1] for reversible motors). Every matrix and vector is
kept in float32 as in PX4 so that thresholds behave identically.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

import numpy as np

f32 = np.float32
NUM_ACTUATORS = 16  # ControlAllocation.hpp: NUM_ACTUATORS
NUM_AXES = 6
FLT_EPSILON = f32(np.finfo(np.float32).eps)
ROLL, PITCH, YAW, THRUST_X, THRUST_Y, THRUST_Z = range(6)  # ActuatorEffectiveness.hpp:75-85

# The flight controller build (CMakeLists.txt:270 CMAKE_CXX_STANDARD 14 with GNU extensions, so
# GCC's default -ffp-contract=fast) fuses "x + a * b" into one FMA on the Cortex-M7 FPU. The
# saturation codes sit on a knife edge at exactly 0, so the replica emulates that contraction:
# a float32 FMA equals the float64 expression a * b + c rounded once to float32 (the product of
# two float32 values is exact in float64).
EMULATE_FMA = True


def fma32(a, b, c):
    """round_f32(a * b + c) with a single rounding when EMULATE_FMA, else the two-rounding
    float32 sequence."""
    if EMULATE_FMA:
        return (
            np.asarray(a, dtype=np.float64) * np.asarray(b, dtype=np.float64)
            + np.asarray(c, dtype=np.float64)
        ).astype(f32)
    return (np.asarray(a, dtype=f32) * np.asarray(b, dtype=f32) + np.asarray(c, dtype=f32)).astype(
        f32
    )


# msg/ControlAllocatorStatus.msg:11-15
ACTUATOR_SATURATION_OK = 0
ACTUATOR_SATURATION_UPPER_DYN = 1
ACTUATOR_SATURATION_UPPER = 2
ACTUATOR_SATURATION_LOWER_DYN = -1
ACTUATOR_SATURATION_LOWER = -2

ACTUATOR_MOTORS_NUM_CONTROLS = 12  # msg/versioned/ActuatorMotors.msg:15
MAX_NUM_MOTORS = 12


class AllocationMethod(IntEnum):
    """ActuatorEffectiveness.hpp:49-54; CA_METHOD (module.yaml:38-49, default 2)."""

    PSEUDO_INVERSE = 0
    SEQUENTIAL_DESATURATION = 1
    AUTO = 2


# ----------------------------------------------------------------------------------------
# matrix library (float32, same accumulation order as MAT:159-173: for i, for k, for j)
# ----------------------------------------------------------------------------------------


def mat_mul_f32(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a (M x N) times b (N x P) in float32 with res(i,k) += a(i,j) * b(j,k) accumulated in
    increasing j (MAT:159-173)."""
    a = np.asarray(a, dtype=f32)
    b = np.asarray(b, dtype=f32)
    res = np.zeros((a.shape[0], b.shape[1]), dtype=f32)
    for j in range(a.shape[1]):
        res = fma32(a[:, j : j + 1], b[j : j + 1, :], res)
    return res


def full_rank_cholesky(a: np.ndarray) -> tuple[np.ndarray, int]:
    """PINV:78-126 fullRankCholesky. a is N x N float32. Returns (L, rank)."""
    a = np.asarray(a, dtype=f32)
    n = a.shape[0]
    # PINV:83: tol = N * typeEpsilon<Type>() * A.diag().max()
    tol = f32(f32(n) * FLT_EPSILON * a.diagonal().max())
    L = np.zeros((n, n), dtype=f32)  # PINV:85
    r = 0  # PINV:87
    for k in range(n):  # PINV:89
        if r == 0:  # PINV:91-94
            for i in range(k, n):
                L[i, r] = a[i, k]
        else:  # PINV:96-106
            for i in range(k, n):
                LL = f32(0.0)
                for j in range(r):
                    LL = fma32(L[i, j], L[k, j], LL)
                L[i, r] = f32(a[i, k] - LL)
        if L[k, r] > tol:  # PINV:109
            L[k, r] = f32(np.sqrt(L[k, r]))  # PINV:110
            if k < n - 1:  # PINV:112-116
                for i in range(k + 1, n):
                    L[i, r] = f32(L[i, r] / L[k, r])
            r = r + 1  # PINV:118
    return L, r  # PINV:123


def inv_f32(a: np.ndarray, rank: int) -> tuple[bool, np.ndarray]:
    """SQM:378-501 inv(A, X, rank): LU inverse of the leading rank x rank block, identity
    elsewhere. Returns (ok, X)."""
    a = np.asarray(a, dtype=f32)
    m = a.shape[0]
    L = np.eye(m, dtype=f32)  # SQM:380-381
    U = a.copy()  # SQM:382
    P = np.eye(m, dtype=f32)  # SQM:383-384
    for n in range(rank):  # SQM:390
        if abs(U[n, n]) < FLT_EPSILON:  # SQM:393
            for i in range(n + 1, rank):  # SQM:395
                if abs(U[i, n]) > FLT_EPSILON:  # SQM:398
                    U[[i, n]] = U[[n, i]]  # SQM:400 swapRows
                    P[[i, n]] = P[[n, i]]  # SQM:401
                    L[[i, n]] = L[[n, i]]  # SQM:402
                    L[:, [i, n]] = L[:, [n, i]]  # SQM:403 swapCols
                    break
        if abs(U[n, n]) < FLT_EPSILON:  # SQM:418-420 failsafe
            return False, np.zeros((m, m), dtype=f32)
        for i in range(n + 1, rank):  # SQM:423-431
            L[i, n] = f32(U[i, n] / U[n, n])
            for k in range(n, rank):
                U[i, k] = fma32(-L[i, n], U[n, k], U[i, k])
    for c in range(rank):  # SQM:440-456 forward substitution (L(i,i) = 1)
        for i in range(rank):
            for j in range(i):
                P[i, c] = fma32(-L[i, j], P[j, c], P[i, c])
    for c in range(rank):  # SQM:464-487 back substitution
        for k in range(rank):
            i = rank - 1 - k
            for j in range(i + 1, rank):
                P[i, c] = fma32(-U[i, j], P[j, c], P[i, c])
            P[i, c] = f32(P[i, c] / U[i, i])
    if not np.all(np.isfinite(P[:rank, :rank])):  # SQM:490-496
        return False, np.zeros((m, m), dtype=f32)
    return True, P  # SQM:499-500


def geninv(g: np.ndarray) -> tuple[bool, np.ndarray]:
    """PINV:24-63 geninv for the M <= N branch (6 x 16 effectiveness). Returns (ok, N x M)."""
    g = np.asarray(g, dtype=f32)
    m, n = g.shape
    if m > n:
        raise ValueError("geninv port covers the M <= N branch only (PINV:29-43)")
    A = mat_mul_f32(g, g.T)  # PINV:30: A = G * G.transpose()
    L, rank = full_rank_cholesky(A)  # PINV:31
    A = mat_mul_f32(L.T, L)  # PINV:33: A = L.transpose() * L
    ok, X = inv_f32(A, rank)  # PINV:36
    if not ok:  # PINV:36-39
        return False, np.zeros((n, m), dtype=f32)
    A = mat_mul_f32(mat_mul_f32(X, X), L.T)  # PINV:42: A = X * X * L.transpose()
    res = mat_mul_f32(g.T, mat_mul_f32(L, A))  # PINV:43: res = G.transpose() * (L * A)
    return True, res


# ----------------------------------------------------------------------------------------
# ControlAllocation (CA)
# ----------------------------------------------------------------------------------------


class ControlAllocation:
    """Port of ControlAllocation (CA.cpp, CA.hpp). Vectors are float32 of length 16."""

    def __init__(self) -> None:
        self._effectiveness = np.zeros((NUM_AXES, NUM_ACTUATORS), dtype=f32)
        self._actuator_trim = np.zeros(NUM_ACTUATORS, dtype=f32)
        self._control_trim = np.zeros(NUM_AXES, dtype=f32)
        self._actuator_min = np.zeros(NUM_ACTUATORS, dtype=f32)  # CA:47 setAll(0)
        self._actuator_max = np.ones(NUM_ACTUATORS, dtype=f32)  # CA:48 setAll(1)
        self._actuator_slew_rate_limit = np.zeros(NUM_ACTUATORS, dtype=f32)
        self._actuator_sp = np.zeros(NUM_ACTUATORS, dtype=f32)
        self._prev_actuator_sp = np.zeros(NUM_ACTUATORS, dtype=f32)
        self._control_sp = np.zeros(NUM_AXES, dtype=f32)
        self._control_allocation_scale = np.ones(NUM_AXES, dtype=f32)  # CA:46 setAll(1)
        self._num_actuators = 0
        self._normalize_rpy = False
        self._had_actuator_failure = False  # CA.hpp:108

    # CA:51-64
    def setEffectivenessMatrix(
        self,
        effectiveness: np.ndarray,
        actuator_trim: np.ndarray,
        linearization_point: np.ndarray,
        num_actuators: int,
        update_normalization_scale: bool,
    ) -> None:
        self._effectiveness = np.asarray(effectiveness, dtype=f32).copy()  # CA:57
        linearization_point_clipped = np.asarray(linearization_point, dtype=f32).copy()  # CA:58
        self.clipActuatorSetpoint(linearization_point_clipped)  # CA:59
        self._actuator_trim = (
            np.asarray(actuator_trim, dtype=f32) + linearization_point_clipped
        ).astype(f32)  # CA:60
        self.clipActuatorSetpoint(self._actuator_trim)  # CA:61
        self._num_actuators = num_actuators  # CA:62
        self._control_trim = mat_mul_f32(self._effectiveness, linearization_point_clipped[:, None])[
            :, 0
        ]  # CA:63

    # CA:67-75
    def setActuatorSetpoint(self, actuator_sp: np.ndarray) -> None:
        self._actuator_sp = np.asarray(actuator_sp, dtype=f32).copy()
        self.clipActuatorSetpoint(self._actuator_sp)

    # CA:78-91 (in place; the module calls the no-argument variant on _actuator_sp)
    def clipActuatorSetpoint(self, actuator: np.ndarray | None = None) -> None:
        if actuator is None:
            actuator = self._actuator_sp
        for i in range(self._num_actuators):
            if self._actuator_max[i] < self._actuator_min[i]:
                actuator[i] = self._actuator_trim[i]
            elif actuator[i] < self._actuator_min[i]:
                actuator[i] = self._actuator_min[i]
            elif actuator[i] > self._actuator_max[i]:
                actuator[i] = self._actuator_max[i]

    # CA:111-126
    def applySlewRateLimit(self, dt: float) -> None:
        dt32 = f32(dt)
        for i in range(self._num_actuators):
            if self._actuator_slew_rate_limit[i] > FLT_EPSILON:
                delta_sp_max = f32(
                    dt32
                    * (self._actuator_max[i] - self._actuator_min[i])
                    / self._actuator_slew_rate_limit[i]
                )
                delta_sp = f32(self._actuator_sp[i] - self._prev_actuator_sp[i])
                if delta_sp > delta_sp_max:
                    self._actuator_sp[i] = f32(self._prev_actuator_sp[i] + delta_sp_max)
                elif delta_sp < -delta_sp_max:
                    self._actuator_sp[i] = f32(self._prev_actuator_sp[i] - delta_sp_max)

    def setControlSetpoint(self, control_sp: np.ndarray) -> None:
        self._control_sp = np.asarray(control_sp, dtype=f32).copy()

    def setActuatorMin(self, v: np.ndarray) -> None:
        self._actuator_min = np.asarray(v, dtype=f32).copy()

    def setActuatorMax(self, v: np.ndarray) -> None:
        self._actuator_max = np.asarray(v, dtype=f32).copy()

    def setSlewRateLimit(self, v: np.ndarray) -> None:
        self._actuator_slew_rate_limit = np.asarray(v, dtype=f32).copy()

    def setNormalizeRPY(self, normalize: bool) -> None:
        self._normalize_rpy = normalize

    def getActuatorSetpoint(self) -> np.ndarray:
        return self._actuator_sp

    def getControlSetpoint(self) -> np.ndarray:
        return self._control_sp

    # CA.hpp:145-146
    def getAllocatedControl(self) -> np.ndarray:
        diff = (self._actuator_sp - self._actuator_trim).astype(f32)
        return (
            mat_mul_f32(self._effectiveness, diff[:, None])[:, 0] * self._control_allocation_scale
        ).astype(f32)

    def allocate(self) -> None:  # pragma: no cover - abstract
        raise NotImplementedError


# ----------------------------------------------------------------------------------------
# ControlAllocationPseudoInverse (CAPI)
# ----------------------------------------------------------------------------------------


class ControlAllocationPseudoInverse(ControlAllocation):
    def __init__(self) -> None:
        super().__init__()
        self._mix = np.zeros((NUM_ACTUATORS, NUM_AXES), dtype=f32)  # CAPI.hpp:63
        self._mix_update_needed = False
        self._normalization_needs_update = False
        self._metric_allocation = False

    # CAPI:44-59
    def setEffectivenessMatrix(
        self,
        effectiveness: np.ndarray,
        actuator_trim: np.ndarray,
        linearization_point: np.ndarray,
        num_actuators: int,
        update_normalization_scale: bool,
    ) -> None:
        super().setEffectivenessMatrix(
            effectiveness,
            actuator_trim,
            linearization_point,
            num_actuators,
            update_normalization_scale,
        )
        self._mix_update_needed = True  # CAPI:52
        self._normalization_needs_update = update_normalization_scale  # CAPI:53
        if self._metric_allocation and update_normalization_scale:  # CAPI:55-58
            self._normalization_needs_update = False

    # CAPI:61-78
    def updatePseudoInverse(self) -> None:
        if self._mix_update_needed:
            _, self._mix = geninv(self._effectiveness)  # CAPI:65 (return value ignored)
            if not self._metric_allocation:  # CAPI:67
                if self._normalization_needs_update and not self._had_actuator_failure:  # CAPI:68
                    self.updateControlAllocationMatrixScale()
                    self._normalization_needs_update = False
                self.normalizeControlAllocationMatrix()  # CAPI:73
            self._mix_update_needed = False

    # CAPI:80-148
    def updateControlAllocationMatrixScale(self) -> None:
        n = self._num_actuators
        scale = self._control_allocation_scale
        if self._normalize_rpy:  # CAPI:83
            num_non_zero_roll_torque = int(
                np.count_nonzero(np.abs(self._mix[:n, 0]) > f32(1e-3))
            )  # CAPI:88-93
            num_non_zero_pitch_torque = int(
                np.count_nonzero(np.abs(self._mix[:n, 1]) > f32(1e-3))
            )  # CAPI:95-97
            roll_norm_scale = f32(1.0)  # CAPI:100
            if num_non_zero_roll_torque > 0:  # CAPI:102-104
                roll_norm_scale = f32(
                    np.sqrt(
                        _norm_squared_f32(self._mix[:, 0]) / f32(num_non_zero_roll_torque / 2.0)
                    )
                )
            pitch_norm_scale = f32(1.0)  # CAPI:106
            if num_non_zero_pitch_torque > 0:  # CAPI:108-110
                pitch_norm_scale = f32(
                    np.sqrt(
                        _norm_squared_f32(self._mix[:, 1]) / f32(num_non_zero_pitch_torque / 2.0)
                    )
                )
            scale[0] = max(roll_norm_scale, pitch_norm_scale)  # CAPI:112 fmaxf
            scale[1] = scale[0]  # CAPI:113
            scale[2] = self._mix[
                :, 2
            ].max()  # CAPI:116: signed max over all 16 rows (Slice.hpp:343-358)
        else:  # CAPI:118-122
            scale[0] = 1.0
            scale[1] = 1.0
            scale[2] = 1.0

        scale[THRUST_Z] = 1.0  # CAPI:127
        for axis_idx in (2, 1, 0):  # CAPI:129
            num_non_zero_thrust = 0
            norm_sum = f32(0.0)
            for i in range(n):  # CAPI:133-140
                norm = f32(abs(self._mix[i, 3 + axis_idx]))
                norm_sum = f32(norm_sum + norm)
                if norm > FLT_EPSILON:
                    num_non_zero_thrust += 1
            if num_non_zero_thrust > 0:  # CAPI:142-143
                scale[3 + axis_idx] = f32(norm_sum / f32(num_non_zero_thrust))
            else:  # CAPI:145-146
                scale[3 + axis_idx] = scale[THRUST_Z]

    # CAPI:150-177
    def normalizeControlAllocationMatrix(self) -> None:
        scale = self._control_allocation_scale
        if scale[0] > FLT_EPSILON:  # CAPI:153-156
            self._mix[:, 0] = (self._mix[:, 0] / scale[0]).astype(f32)
            self._mix[:, 1] = (self._mix[:, 1] / scale[1]).astype(f32)
        if scale[2] > FLT_EPSILON:  # CAPI:158-160
            self._mix[:, 2] = (self._mix[:, 2] / scale[2]).astype(f32)
        if scale[3] > FLT_EPSILON:  # CAPI:162-166
            self._mix[:, 3] = (self._mix[:, 3] / scale[3]).astype(f32)
            self._mix[:, 4] = (self._mix[:, 4] / scale[4]).astype(f32)
            self._mix[:, 5] = (self._mix[:, 5] / scale[5]).astype(f32)
        n = self._num_actuators
        small = np.abs(self._mix[:n, :]) < f32(1e-3)  # CAPI:170-176
        self._mix[:n, :][small] = 0.0

    # CAPI:179-189
    def allocate(self) -> None:
        self.updatePseudoInverse()  # CAPI:183
        self._prev_actuator_sp = self._actuator_sp.copy()  # CAPI:185
        # CAPI:188: _actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim)
        diff = (self._control_sp - self._control_trim).astype(f32)
        self._actuator_sp = (
            self._actuator_trim + mat_mul_f32(self._mix, diff[:, None])[:, 0]
        ).astype(f32)


def _norm_squared_f32(col: np.ndarray) -> f32:
    # Slice.hpp:319-331 norm_squared: accumulate val*val over all rows in order
    acc = f32(0.0)
    for v in col:
        acc = f32(acc + v * v)
    return acc


# ----------------------------------------------------------------------------------------
# ControlAllocationSequentialDesaturation (CASD)
# ----------------------------------------------------------------------------------------


class ControlAllocationSequentialDesaturation(ControlAllocationPseudoInverse):
    MINIMUM_YAW_MARGIN = f32(0.15)  # CASD.hpp:63

    def __init__(self, mc_airmode: int = 0) -> None:
        super().__init__()
        # CASD.hpp:129-131 (ParamInt MC_AIRMODE, src/lib/mixer_module/params.c:18 default 0)
        self._param_mc_airmode = int(mc_airmode)

    # CASD:44-65
    def allocate(self) -> None:
        self.updatePseudoInverse()  # CASD:48
        self._prev_actuator_sp = self._actuator_sp.copy()  # CASD:50
        if self._param_mc_airmode == 1:  # CASD:52-54
            self.mixAirmodeRP()
        elif self._param_mc_airmode == 2:  # CASD:56-58
            self.mixAirmodeRPY()
        else:  # CASD:60-62
            self.mixAirmodeDisabled()

    # CASD:67-86
    def desaturateActuators(
        self, actuator_sp: np.ndarray, desaturation_vector: np.ndarray, increase_only: bool = False
    ) -> None:
        n = self._num_actuators
        gain = self.computeDesaturationGain(desaturation_vector, actuator_sp)  # CASD:71
        if increase_only and gain < f32(0.0):  # CASD:73-75
            return
        actuator_sp[:n] = fma32(gain, desaturation_vector[:n], actuator_sp[:n])  # CASD:77-79
        gain = f32(
            f32(0.5) * self.computeDesaturationGain(desaturation_vector, actuator_sp)
        )  # CASD:81
        actuator_sp[:n] = fma32(gain, desaturation_vector[:n], actuator_sp[:n])  # CASD:83-85

    # CASD:88-119
    def computeDesaturationGain(
        self, desaturation_vector: np.ndarray, actuator_sp: np.ndarray
    ) -> f32:
        k_min = f32(0.0)  # CASD:91
        k_max = f32(0.0)  # CASD:92
        for i in range(self._num_actuators):  # CASD:94
            if abs(desaturation_vector[i]) < f32(0.2):  # CASD:96-98 weak effectiveness: skip
                continue
            if actuator_sp[i] < self._actuator_min[i]:  # CASD:100-106
                k = f32((self._actuator_min[i] - actuator_sp[i]) / desaturation_vector[i])
                if k < k_min:
                    k_min = k
                if k > k_max:
                    k_max = k
            if actuator_sp[i] > self._actuator_max[i]:  # CASD:108-114
                k = f32((self._actuator_max[i] - actuator_sp[i]) / desaturation_vector[i])
                if k < k_min:
                    k_min = k
                if k > k_max:
                    k_max = k
        return f32(k_min + k_max)  # CASD:118

    def _mix_axes(self, axes: tuple[int, ...]) -> None:
        """_actuator_sp(i) = _actuator_trim(i) + sum over the given axes, in float32 and in
        the listed order (CASD:130-134, 155-160, 184-188)."""
        n = self._num_actuators
        sp = self._actuator_trim[:n].copy()
        for ax in axes:
            sp = fma32(self._mix[:n, ax], f32(self._control_sp[ax] - self._control_trim[ax]), sp)
        self._actuator_sp[:n] = sp

    # CASD:121-143
    def mixAirmodeRP(self) -> None:
        self._mix_axes((ROLL, PITCH, THRUST_X, THRUST_Y, THRUST_Z))  # CASD:129-135 (no yaw)
        thrust_z = self._mix[:, THRUST_Z].copy()  # CASD:136
        self.desaturateActuators(self._actuator_sp, thrust_z)  # CASD:139
        self.mixYaw()  # CASD:142

    # CASD:145-171
    def mixAirmodeRPY(self) -> None:
        self._mix_axes((ROLL, PITCH, YAW, THRUST_X, THRUST_Y, THRUST_Z))  # CASD:154-161
        thrust_z = self._mix[:, THRUST_Z].copy()  # CASD:162
        yaw = self._mix[:, YAW].copy()  # CASD:163
        self.desaturateActuators(self._actuator_sp, thrust_z)  # CASD:166
        self.desaturateActuators(self._actuator_sp, yaw)  # CASD:170 prioritise roll/pitch over yaw

    # CASD:173-204
    def mixAirmodeDisabled(self) -> None:
        self._mix_axes((ROLL, PITCH, THRUST_X, THRUST_Y, THRUST_Z))  # CASD:183-189 (no yaw)
        thrust_z = self._mix[:, THRUST_Z].copy()  # CASD:190
        roll = self._mix[:, ROLL].copy()  # CASD:191
        pitch = self._mix[:, PITCH].copy()  # CASD:192
        self.desaturateActuators(self._actuator_sp, thrust_z, True)  # CASD:196 only reduce thrust
        self.desaturateActuators(self._actuator_sp, roll)  # CASD:199
        self.desaturateActuators(self._actuator_sp, pitch)  # CASD:200
        self.mixYaw()  # CASD:203

    # CASD:206-228
    def mixYaw(self) -> None:
        n = self._num_actuators
        # CASD:213-217: add yaw to outputs
        self._actuator_sp[:n] = fma32(
            self._mix[:n, YAW],
            f32(self._control_sp[YAW] - self._control_trim[YAW]),
            self._actuator_sp[:n],
        )
        yaw = self._mix[:, YAW].copy()
        thrust_z = self._mix[:, THRUST_Z].copy()
        max_prev = self._actuator_max.copy()  # CASD:221
        # CASD:222: _actuator_max += (_actuator_max - _actuator_min) * MINIMUM_YAW_MARGIN
        self._actuator_max = fma32(
            (self._actuator_max - self._actuator_min).astype(f32),
            self.MINIMUM_YAW_MARGIN,
            self._actuator_max,
        )
        self.desaturateActuators(self._actuator_sp, yaw)  # CASD:223
        self._actuator_max = max_prev  # CASD:224
        self.desaturateActuators(self._actuator_sp, thrust_z, True)  # CASD:227 reduce thrust only


# ----------------------------------------------------------------------------------------
# ControlAllocator module glue (MOD)
# ----------------------------------------------------------------------------------------


def resolve_allocation_method(ca_method: int) -> AllocationMethod:
    """MOD:139, 172-174: CA_METHOD AUTO (2) resolves to the airframe's desired method, which
    is SEQUENTIAL_DESATURATION for the multirotor (ActuatorEffectivenessMultirotor.hpp:47-50)."""
    configured = AllocationMethod(int(ca_method))
    if configured == AllocationMethod.AUTO:
        return AllocationMethod.SEQUENTIAL_DESATURATION
    return configured


def zero_weak_rows(effectiveness: np.ndarray, num_actuators: int) -> np.ndarray:
    """MOD:571-583: set a whole axis row to 0 unless some entry among the first
    num_actuators columns has |entry| > 0.05 (strict)."""
    m = np.asarray(effectiveness, dtype=f32).copy()
    for n in range(NUM_AXES):
        all_entries_small = True
        for col in range(num_actuators):
            if abs(m[n, col]) > f32(0.05):
                all_entries_small = False
        if all_entries_small:
            m[n, :] = 0.0
    return m


@dataclass
class AllocatorStatus:
    """control_allocator_status fields (MOD:596-647): unallocated values are in the
    normalised control units of the setpoint; saturation codes per ControlAllocatorStatus.msg."""

    unallocated_torque: np.ndarray
    unallocated_thrust: np.ndarray
    torque_setpoint_achieved: bool
    thrust_setpoint_achieved: bool
    actuator_saturation: np.ndarray  # int8[16]


class ControlAllocatorReplica:
    """Stateful replica of the ControlAllocator module for one effectiveness configuration.

    effectiveness: 6 x N (N <= 16) UNZEROED matrix from geometry.effectiveness_matrix, with
    the units described there. Parameters: CA_METHOD, MC_AIRMODE, CA_R_REV (bitmask),
    CA_Rn_SLEW (seconds per full range, list of N).
    """

    def __init__(
        self,
        effectiveness: np.ndarray,
        *,
        ca_method: int = 2,
        mc_airmode: int = 0,
        r_rev: int = 0,
        slew_rates: list[float] | np.ndarray | None = None,
    ) -> None:
        eff = np.asarray(effectiveness, dtype=f32)
        if eff.shape[0] != NUM_AXES or eff.shape[1] > NUM_ACTUATORS:
            raise ValueError("effectiveness must be 6 x N with N <= 16")
        num_actuators = eff.shape[1]
        if num_actuators > MAX_NUM_MOTORS:
            raise ValueError("MOD:503-507: at most 12 motors")  # MOD:503-507 'Too many motors'
        full = np.zeros((NUM_AXES, NUM_ACTUATORS), dtype=f32)
        full[:, :num_actuators] = eff
        self.num_actuators = num_actuators
        self.method = resolve_allocation_method(ca_method)
        self.r_rev = int(r_rev)

        slew = np.zeros(MAX_NUM_MOTORS, dtype=f32)
        if slew_rates is not None:
            s = np.asarray(slew_rates, dtype=f32).ravel()
            slew[: len(s)] = s
        # MOD:108-113: _has_slew_rate |= slew_rate_motors[i] > FLT_EPSILON
        self._has_slew_rate = bool(np.any(slew > FLT_EPSILON))

        # MOD:180-187 method switch, MOD:195 setNormalizeRPY (true, AEM.hpp:52-55)
        if self.method == AllocationMethod.PSEUDO_INVERSE:
            self.alloc: ControlAllocationPseudoInverse = ControlAllocationPseudoInverse()
        else:
            self.alloc = ControlAllocationSequentialDesaturation(mc_airmode)
        self.alloc.setNormalizeRPY(True)
        self.alloc.setActuatorSetpoint(np.zeros(NUM_ACTUATORS, dtype=f32))  # MOD:196

        # MOD:479-533: min/max/slew, default-constructed (zero) for unused slots
        minimum = np.zeros(NUM_ACTUATORS, dtype=f32)
        maximum = np.zeros(NUM_ACTUATORS, dtype=f32)
        slew_rate = np.zeros(NUM_ACTUATORS, dtype=f32)
        for idx in range(num_actuators):
            if self.r_rev & (1 << idx):  # MOD:508-513
                minimum[idx] = -1.0
            else:
                minimum[idx] = 0.0
            slew_rate[idx] = slew[idx]  # MOD:515
            maximum[idx] = 1.0  # MOD:532
        self.alloc.setActuatorMin(minimum)  # MOD:561
        self.alloc.setActuatorMax(maximum)  # MOD:562
        self.alloc.setSlewRateLimit(slew_rate)  # MOD:563

        # MOD:571-583 weak-row rule, then MOD:586-588 with trim = linearization_point = 0
        # (Configuration config{} value-initialised, MOD:466) and update_normalization_scale
        # = true (reason CONFIGURATION_UPDATE from parameters_updated, MOD:133).
        self.effectiveness = zero_weak_rows(full, num_actuators)
        self.alloc.setEffectivenessMatrix(
            self.effectiveness,
            np.zeros(NUM_ACTUATORS, dtype=f32),
            np.zeros(NUM_ACTUATORS, dtype=f32),
            num_actuators,
            True,
        )
        self.alloc.updatePseudoInverse()  # normally lazily inside allocate(); do it now

    @property
    def mix(self) -> np.ndarray:
        """Normalised 16 x 6 mixing matrix _mix (CAPI.hpp:63)."""
        return self.alloc._mix

    @property
    def scale(self) -> np.ndarray:
        """_control_allocation_scale (6), see CAPI:80-148."""
        return self.alloc._control_allocation_scale

    def step(self, torque_sp: np.ndarray, thrust_sp: np.ndarray, dt: float = 0.004) -> np.ndarray:
        """One allocator run (MOD:376-442) for a torque setpoint (normalised, FRD) and the latest
        thrust setpoint (normalised, FRD). dt in seconds is clamped to [0.0002, 0.02] (MOD:378)
        and only matters when a slew rate is configured. Returns the clipped actuator setpoint
        (16, float64)."""
        dt = min(max(float(dt), 0.0002), 0.02)  # MOD:378
        c = np.zeros(NUM_AXES, dtype=f32)  # MOD:406-411
        c[0:3] = np.asarray(torque_sp, dtype=f32)
        c[3:6] = np.asarray(thrust_sp, dtype=f32)
        self.alloc.setControlSetpoint(c)  # MOD:429
        self.alloc.allocate()  # MOD:432
        # MOD:433-435 allocateAuxilaryControls / updateSetpoint: no-ops for the multirotor
        if self._has_slew_rate:  # MOD:437-439
            self.alloc.applySlewRateLimit(dt)
        self.alloc.clipActuatorSetpoint()  # MOD:441
        return self.alloc.getActuatorSetpoint().astype(np.float64)

    def status(self) -> AllocatorStatus:
        """control_allocator_status from the current (clipped) setpoint, MOD:596-647."""
        allocated = self.alloc.getAllocatedControl()  # MOD:605
        unallocated = (self.alloc.getControlSetpoint() - allocated).astype(f32)  # MOD:608-609
        unalloc_torque = unallocated[0:3].astype(np.float64)  # MOD:610-612
        unalloc_thrust = unallocated[3:6].astype(np.float64)  # MOD:613-615
        # MOD:621-626: achieved if norm_squared < 1e-6
        torque_ok = bool(_norm_squared_f32(unallocated[0:3]) < f32(1e-6))
        thrust_ok = bool(_norm_squared_f32(unallocated[3:6]) < f32(1e-6))
        sp = self.alloc.getActuatorSetpoint()
        amin = self.alloc._actuator_min
        amax = self.alloc._actuator_max
        sat = np.zeros(NUM_ACTUATORS, dtype=np.int8)
        for i in range(NUM_ACTUATORS):  # MOD:633-640, all 16 slots
            if sp[i] > (amax[i] - FLT_EPSILON):
                sat[i] = ACTUATOR_SATURATION_UPPER
            elif sp[i] < (amin[i] + FLT_EPSILON):
                sat[i] = ACTUATOR_SATURATION_LOWER
        return AllocatorStatus(unalloc_torque, unalloc_thrust, torque_ok, thrust_ok, sat)

    def actuator_motors(self, stopped_motors: int = 0) -> np.ndarray:
        """actuator_motors.control[12] (MOD:649-691): the clipped setpoint for i < num
        actuators (NaN if non-finite or stopped), NaN for the remaining slots."""
        sp = self.alloc.getActuatorSetpoint()
        control = np.full(ACTUATOR_MOTORS_NUM_CONTROLS, np.nan)
        for i in range(min(self.num_actuators, ACTUATOR_MOTORS_NUM_CONTROLS)):  # MOD:673-682
            control[i] = sp[i] if np.isfinite(sp[i]) else np.nan
            if stopped_motors & (1 << i):
                control[i] = np.nan
        return control


def motor_output_values(
    control: np.ndarray, thr_mdl_fac: float = 0.0, reversible_flags: int = 0
) -> np.ndarray:
    """FunctionMotors::updateValues (FM:81-120): THR_MDL_FAC inverse thrust model
    (motor_params.c:47-57, default 0) and the non-reversible remap [0, 1] -> [-1, 1].
    Not part of the allocator: the logged actuator_motors.control is its input."""
    values = np.asarray(control, dtype=f32).copy()
    thrust_factor = f32(thr_mdl_fac)
    if thrust_factor > FLT_EPSILON and thrust_factor <= f32(1.0):  # FM:83
        a = thrust_factor  # FM:86
        b = f32(f32(1.0) - thrust_factor)  # FM:87
        tmp1 = f32(b / (f32(2.0) * a))  # FM:90
        tmp2 = f32(b * b / (f32(4.0) * a * a))  # FM:91
        for i in range(values.shape[0]):  # FM:93-105
            ctrl = values[i]
            if np.isfinite(ctrl):
                if ctrl > FLT_EPSILON:
                    values[i] = f32(-tmp1 + np.sqrt(f32(tmp2 + ctrl / a)))
                elif ctrl < -FLT_EPSILON:
                    values[i] = f32(tmp1 - np.sqrt(f32(tmp2 - ctrl / a)))
    for i in range(values.shape[0]):  # FM:108-118
        if (reversible_flags & (1 << i)) == 0:
            if values[i] < -FLT_EPSILON:
                values[i] = np.nan
            else:
                values[i] = f32(values[i] * f32(2.0) - f32(1.0))
    return values.astype(np.float64)


@dataclass
class AllocationResult:
    """One-shot allocation result. actuator_sp: clipped normalised motor commands (N);
    unallocated_*: normalised control units (setpoint minus achieved); saturation: int8[16]
    codes as ControlAllocatorStatus.msg; actuator_motors: control[12] with NaN padding."""

    actuator_sp: np.ndarray
    unallocated_torque: np.ndarray
    unallocated_thrust: np.ndarray
    torque_setpoint_achieved: bool
    thrust_setpoint_achieved: bool
    saturation: np.ndarray
    actuator_motors: np.ndarray


def allocate(
    effectiveness: np.ndarray,
    torque_sp: np.ndarray,
    thrust_sp: np.ndarray,
    *,
    ca_method: int = 2,
    mc_airmode: int = 0,
    r_rev: int = 0,
    replica: ControlAllocatorReplica | None = None,
) -> AllocationResult:
    """Allocate one setpoint (torque normalised FRD, thrust normalised FRD, thrust z < 0 up)
    on a 6 x N effectiveness matrix exactly as the PX4 module would after a parameter update.
    Pass a prebuilt replica to reuse the pseudo-inverse across many setpoints (slew is not
    applied because dt is unknown in a one-shot call; with CA_Rn_SLEW = 0 it never is)."""
    rep = replica or ControlAllocatorReplica(
        effectiveness, ca_method=ca_method, mc_airmode=mc_airmode, r_rev=r_rev
    )
    sp = rep.step(torque_sp, thrust_sp)
    st = rep.status()
    return AllocationResult(
        actuator_sp=sp[: rep.num_actuators],
        unallocated_torque=st.unallocated_torque,
        unallocated_thrust=st.unallocated_thrust,
        torque_setpoint_achieved=st.torque_setpoint_achieved,
        thrust_setpoint_achieved=st.thrust_setpoint_achieved,
        saturation=st.actuator_saturation,
        actuator_motors=rep.actuator_motors(),
    )
