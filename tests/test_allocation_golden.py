"""Golden test of the allocator replica against logged PX4 data (PLAN.md section 6/M2).

Parameters come from ULog.initial_parameters. For every logged actuator_motors and
control_allocator_status sample the replica is fed the most recent preceding
vehicle_torque_setpoint and vehicle_thrust_setpoint samples (zero-order hold; the logger ran
at 50 Hz for setpoints, 10 Hz for motors, 5 Hz for status, the allocator much faster), so the
residual contains the setpoint change between the logged setpoint and the one that actually
triggered the logged output. Per-sample diagnostics go to TILTLAB_DIAG_DIR (default: the
system temp directory), never into the repository.
"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest
from pyulog import ULog

from tiltlab.core.allocation import ACTUATOR_SATURATION_LOWER, ControlAllocatorReplica
from tiltlab.core.geometry import compute_effectiveness_matrix, rotors_from_px4_params

FIXTURES = Path(__file__).parent / "fixtures"
LOGS = {
    "log36": FIXTURES / "log_36_2026-9-9-14-11-36.ulg",
    "log40": FIXTURES / "log_40_2026-9-9-15-09-54.ulg",
}
TOPICS = [
    "actuator_motors",
    "vehicle_torque_setpoint",
    "vehicle_thrust_setpoint",
    "control_allocator_status",
]
N_MOTORS = 10


def _diag_dir() -> Path:
    d = Path(os.environ.get("TILTLAB_DIAG_DIR", Path(tempfile.gettempdir()) / "tiltlab_golden"))
    d.mkdir(parents=True, exist_ok=True)
    return d


@dataclass
class Replay:
    name: str
    t0_us: int
    motor_err: np.ndarray  # (n_motor_samples, 10) replica - logged
    motor_t: np.ndarray
    status_t: np.ndarray
    status_yaw_sp: np.ndarray
    unalloc_torque_logged: np.ndarray
    unalloc_torque_replica: np.ndarray
    unalloc_thrust_logged: np.ndarray
    unalloc_thrust_replica: np.ndarray
    sat_logged: np.ndarray  # (n, 16)
    sat_replica: np.ndarray

    @property
    def motor_rms(self) -> float:
        return float(np.sqrt(np.mean(self.motor_err**2)))

    @property
    def motor_max(self) -> float:
        return float(np.abs(self.motor_err).max())

    @property
    def sat_match_rate(self) -> float:
        return float(np.mean(self.sat_logged[:, :N_MOTORS] == self.sat_replica[:, :N_MOTORS]))

    @property
    def sat_match_rate_all16(self) -> float:
        return float(np.mean(self.sat_logged == self.sat_replica))

    def status_index_at(self, t_rel_s: float) -> int:
        return int(np.argmin(np.abs((self.status_t - self.t0_us) / 1e6 - t_rel_s)))


def _zoh_index(times: np.ndarray, t: int) -> int:
    return int(np.searchsorted(times, t, side="right") - 1)


def replay_log(name: str, path: Path) -> Replay:
    ulog = ULog(str(path), message_name_filter_list=TOPICS)
    params = {
        k: (int(v) if isinstance(v, int | np.integer) else float(v))
        for k, v in ulog.initial_parameters.items()
    }
    data = {ds.name: ds.data for ds in ulog.data_list}
    tq, th, am, cas = (
        data["vehicle_torque_setpoint"],
        data["vehicle_thrust_setpoint"],
        data["actuator_motors"],
        data["control_allocator_status"],
    )

    eff, n = compute_effectiveness_matrix(rotors_from_px4_params(params))
    assert n == N_MOTORS
    replica = ControlAllocatorReplica(
        eff[:, :n],
        ca_method=params["CA_METHOD"],
        mc_airmode=params["MC_AIRMODE"],
        r_rev=params["CA_R_REV"],
        slew_rates=[params.get(f"CA_R{i}_SLEW", 0.0) for i in range(12)],
    )

    def setpoints(t: int) -> tuple[np.ndarray, np.ndarray] | None:
        it, ih = _zoh_index(tq["timestamp"], t), _zoh_index(th["timestamp"], t)
        if it < 0 or ih < 0:
            return None
        torque = np.array([tq[f"xyz[{j}]"][it] for j in range(3)])
        thrust = np.array([th[f"xyz[{j}]"][ih] for j in range(3)])
        return torque, thrust

    motor_err, motor_t, per_sample = [], [], []
    for k in range(len(am["timestamp"])):
        t = int(am["timestamp"][k])
        sp = setpoints(t)
        logged = np.array([am[f"control[{j}]"][k] for j in range(N_MOTORS)])
        if sp is None or not np.all(np.isfinite(logged)):
            continue
        out = replica.step(sp[0], sp[1])[:N_MOTORS]
        motor_err.append(out - logged)
        motor_t.append(t)
        per_sample.append(
            {
                "t_s": (t - ulog.start_timestamp) / 1e6,
                "torque": sp[0].tolist(),
                "thrust": sp[1].tolist(),
                "logged": logged.tolist(),
                "replica": out.tolist(),
            }
        )

    st_t, yaw_sp, ut_l, ut_r, uh_l, uh_r, s_l, s_r, st_rows = [], [], [], [], [], [], [], [], []
    for k in range(len(cas["timestamp"])):
        t = int(cas["timestamp"][k])
        sp = setpoints(t)
        if sp is None:
            continue
        replica.step(sp[0], sp[1])
        st = replica.status()
        st_t.append(t)
        yaw_sp.append(sp[0][2])
        ut_l.append([cas[f"unallocated_torque[{j}]"][k] for j in range(3)])
        uh_l.append([cas[f"unallocated_thrust[{j}]"][k] for j in range(3)])
        s_l.append([cas[f"actuator_saturation[{j}]"][k] for j in range(16)])
        ut_r.append(st.unallocated_torque)
        uh_r.append(st.unallocated_thrust)
        s_r.append(st.actuator_saturation)
        st_rows.append(
            {
                "t_s": (t - ulog.start_timestamp) / 1e6,
                "torque": sp[0].tolist(),
                "thrust": sp[1].tolist(),
                "unalloc_torque_logged": ut_l[-1],
                "unalloc_torque_replica": st.unallocated_torque.tolist(),
                "unalloc_thrust_logged": uh_l[-1],
                "unalloc_thrust_replica": st.unallocated_thrust.tolist(),
                "sat_logged": [int(v) for v in s_l[-1]],
                "sat_replica": [int(v) for v in st.actuator_saturation],
            }
        )

    rep = Replay(
        name=name,
        t0_us=int(ulog.start_timestamp),
        motor_err=np.array(motor_err),
        motor_t=np.array(motor_t),
        status_t=np.array(st_t),
        status_yaw_sp=np.array(yaw_sp),
        unalloc_torque_logged=np.array(ut_l, dtype=float),
        unalloc_torque_replica=np.array(ut_r, dtype=float),
        unalloc_thrust_logged=np.array(uh_l, dtype=float),
        unalloc_thrust_replica=np.array(uh_r, dtype=float),
        sat_logged=np.array(s_l, dtype=int),
        sat_replica=np.array(s_r, dtype=int),
    )
    diag = {
        "log": path.name,
        "params_ca": {
            k: v for k, v in params.items() if k.startswith(("CA_", "MC_AIRMODE", "THR_MDL"))
        },
        "scale": replica.scale.tolist(),
        "mix": replica.mix[:N_MOTORS].tolist(),
        "motor_rms": rep.motor_rms,
        "motor_max": rep.motor_max,
        "saturation_match_rate_motors": rep.sat_match_rate,
        "saturation_match_rate_all16": rep.sat_match_rate_all16,
        "motors": per_sample,
        "status": st_rows,
    }
    (_diag_dir() / f"golden_{name}.json").write_text(
        json.dumps(diag, indent=1, default=lambda o: o.item()), encoding="utf-8"
    )
    return rep


@pytest.fixture(scope="module")
def replays() -> dict[str, Replay]:
    out = {name: replay_log(name, path) for name, path in LOGS.items()}
    print("\n log    | motors | motor RMS | motor max | status | sat match (0..9) | sat match (16)")
    for r in out.values():
        print(
            f" {r.name:6s} | {len(r.motor_err):6d} | {r.motor_rms:9.5f} | {r.motor_max:9.5f} |"
            f" {len(r.status_t):6d} | {r.sat_match_rate:16.4f} | {r.sat_match_rate_all16:.4f}"
        )
    return out


@pytest.mark.parametrize("name", list(LOGS))
def test_motor_rms_error(replays: dict[str, Replay], name: str) -> None:
    r = replays[name]
    assert len(r.motor_err) > 400
    assert r.motor_rms < 0.02, (
        f"{name}: motor RMS error {r.motor_rms:.5f} >= 0.02 (max {r.motor_max:.5f})"
    )


@pytest.mark.parametrize("name", list(LOGS))
def test_saturation_codes_match(replays: dict[str, Replay], name: str) -> None:
    r = replays[name]
    assert set(np.unique(r.sat_logged[:, N_MOTORS:])) == {2}, (
        "unused slots are logged as UPPER (MOD:634)"
    )
    assert set(np.unique(r.sat_replica[:, N_MOTORS:])) == {2}
    assert r.sat_match_rate >= 0.95, f"{name}: saturation match rate {r.sat_match_rate:.4f}"


def test_log36_unallocated_thrust_at_33s(replays: dict[str, Replay]) -> None:
    r = replays["log36"]
    k = r.status_index_at(33.0)
    logged = r.unalloc_thrust_logged[k, 2]
    replica = r.unalloc_thrust_replica[k, 2]
    print(f"\n log36 unallocated thrust z at 33 s: logged {logged:.4f}, replica {replica:.4f}")
    assert logged == pytest.approx(-0.9355, abs=0.01), "fixture fact from README changed"
    assert np.sign(replica) == np.sign(logged)
    assert abs(replica - logged) <= 0.05 * abs(logged), (
        f"replica {replica:.4f} vs logged {logged:.4f}"
    )
    assert list(r.sat_replica[k, :8]) == [ACTUATOR_SATURATION_LOWER] * 8, r.sat_replica[k].tolist()
    assert list(r.sat_logged[k, :8]) == [ACTUATOR_SATURATION_LOWER] * 8
    # residual of unallocated thrust z over all status samples (reported, not asserted)
    dz = r.unalloc_thrust_replica[:, 2] - r.unalloc_thrust_logged[:, 2]
    print(
        f" log36 unallocated thrust z residual: rms {np.sqrt(np.mean(dz**2)):.4f},"
        f" max {np.abs(dz).max():.4f}"
    )


def test_log40_yaw_unallocated_fraction(replays: dict[str, Replay]) -> None:
    r = replays["log40"]
    mask = np.abs(r.status_yaw_sp) > 0.05
    assert mask.sum() > 50
    frac_logged = float(
        np.mean(np.abs(r.unalloc_torque_logged[mask, 2]) / np.abs(r.status_yaw_sp[mask]))
    )
    frac_replica = float(
        np.mean(np.abs(r.unalloc_torque_replica[mask, 2]) / np.abs(r.status_yaw_sp[mask]))
    )
    print(
        f"\n log40 |unallocated yaw| / |yaw sp| over {int(mask.sum())} samples:"
        f" logged {frac_logged:.4f}, replica {frac_replica:.4f}"
    )
    assert frac_logged == pytest.approx(0.778, abs=0.03), "fixture fact from README changed"
    assert abs(frac_replica - frac_logged) <= 0.05 * frac_logged, (
        f"{frac_replica:.4f} vs {frac_logged:.4f}"
    )
