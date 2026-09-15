"""One line per ulog: the numbers that decide a gain change, side by side.

    uv run --project backend python .claude/skills/pid-tuning/scripts/tune_table.py exports/logs/run*.ulg [--csv out.csv]

Hover window: from the first time the vehicle is more than 1 m up until the last time it is,
trimmed by 3 s at both ends (takeoff and landing transients out). Per axis: attitude peak to
peak in degrees, dominant frequency of the attitude in Hz, RMS rate error in deg/s (rate
setpoint minus measured rate), normalised torque demand peak (1 = full authority) and the
fraction of the window with any motor saturated. Then horizontal drift (max distance from the
window mean) and altitude standard deviation in metres, mean motor command, and the gains the
log ran with. Frames: PX4 body FRD, degrees here because this is a table for people.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog

GAINS = (
    "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_PITCHRATE_P", "MC_PITCHRATE_I",
    "MC_PITCHRATE_D", "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P",
    "MC_RR_INT_LIM", "MC_PR_INT_LIM", "MPC_XY_VEL_P_ACC", "MPC_XY_P", "MPC_Z_VEL_P_ACC",
    "MPC_THR_HOVER", "MPC_TILTMAX_AIR",
)  # fmt: skip
AXES = ("roll", "pitch", "yaw")


def topic(u: ULog, name: str, instance: int = 0):
    for d in u.data_list:
        if d.name == name and d.multi_id == instance:
            return d
    return None


def euler_deg(q: np.ndarray) -> np.ndarray:
    q0, q1, q2, q3 = q.T
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1))
    yaw = np.unwrap(np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)))
    return np.degrees(np.stack([roll, pitch, yaw], 1))


def dominant_hz(x: np.ndarray, t: np.ndarray) -> float:
    if len(x) < 16:
        return 0.0
    x = x - x.mean()
    dt = float(np.median(np.diff(t)))
    spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    f = np.fft.rfftfreq(len(x), dt)
    spec[f < 0.1] = 0.0
    return float(f[int(np.argmax(spec))])


def resample(t_src: np.ndarray, x: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    return np.interp(t_dst, t_src, x)


def analyse(path: Path) -> dict[str, object]:
    u = ULog(str(path))
    p = u.initial_parameters
    lp = topic(u, "vehicle_local_position")
    att = topic(u, "vehicle_attitude")
    if lp is None or att is None:
        return {"log": path.name, "note": "no position or attitude topic"}
    t_lp = lp.data["timestamp"] / 1e6
    up = -lp.data["z"] > 1.0
    if not up.any():
        return {"log": path.name, "note": "never above 1 m", **{g: p.get(g) for g in GAINS}}
    t0, t1 = t_lp[up][0] + 3.0, t_lp[up][-1] - 3.0
    if t1 - t0 < 5.0:
        return {"log": path.name, "note": f"hover window {t1 - t0:.1f} s too short", **{g: p.get(g) for g in GAINS}}
    row: dict[str, object] = {"log": path.name, "hover_s": round(t1 - t0, 1)}

    t_att = att.data["timestamp"] / 1e6
    w = (t_att >= t0) & (t_att <= t1)
    eul = euler_deg(np.stack([att.data[f"q[{i}]"] for i in range(4)], 1))[w]
    for k, ax in enumerate(AXES):
        row[f"{ax}_pp_deg"] = round(float(np.ptp(eul[:, k])), 2)
        row[f"{ax}_hz"] = round(dominant_hz(eul[:, k], t_att[w]), 2)

    rates = topic(u, "vehicle_angular_velocity")
    rsp = topic(u, "vehicle_rates_setpoint")
    if rates is not None and rsp is not None:
        t_r = rates.data["timestamp"] / 1e6
        wr = (t_r >= t0) & (t_r <= t1)
        t_s = rsp.data["timestamp"] / 1e6
        for k, ax in enumerate(AXES):
            sp = resample(t_s, rsp.data[AXES[k]], t_r[wr])
            err = np.degrees(sp - rates.data[f"xyz[{k}]"][wr])
            row[f"{ax}_rate_err_rms"] = round(float(np.sqrt(np.mean(err**2))), 2)

    tq = topic(u, "vehicle_torque_setpoint")
    if tq is not None:
        t_q = tq.data["timestamp"] / 1e6
        wq = (t_q >= t0) & (t_q <= t1)
        for k, ax in enumerate(AXES):
            row[f"{ax}_tq_peak"] = round(float(np.abs(tq.data[f"xyz[{k}]"][wq]).max()), 3)

    cas = topic(u, "control_allocator_status")
    if cas is not None:
        t_c = cas.data["timestamp"] / 1e6
        wc = (t_c >= t0) & (t_c <= t1)
        sat = np.zeros(int(wc.sum()), dtype=bool)
        n_rot = int(p.get("CA_ROTOR_COUNT", 16) or 16)  # unused slots report code 2 forever
        for i in range(n_rot):
            key = f"actuator_saturation[{i}]"
            if key in cas.data:
                sat |= cas.data[key][wc] != 0
        row["sat_frac"] = round(float(sat.mean()) if len(sat) else 0.0, 3)

    wl = (t_lp >= t0) & (t_lp <= t1)
    xy = np.stack([lp.data["x"][wl], lp.data["y"][wl]], 1)
    row["xy_drift_m"] = round(float(np.linalg.norm(xy - xy.mean(0), axis=1).max()), 2)
    row["alt_std_m"] = round(float(np.std(lp.data["z"][wl])), 3)

    mot = topic(u, "actuator_motors")
    if mot is not None:
        t_m = mot.data["timestamp"] / 1e6
        wm = (t_m >= t0) & (t_m <= t1)
        cols = [mot.data[f"control[{i}]"][wm] for i in range(12) if f"control[{i}]" in mot.data]
        m = np.stack(cols, 1)
        m = m[:, np.isfinite(m).all(0)]
        row["motor_mean"] = round(float(np.nanmean(m)), 3)
        row["motor_max"] = round(float(np.nanmax(m)), 3)
    for g in GAINS:
        v = p.get(g)
        row[g] = None if v is None else round(float(v), 4)
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--csv", help="also write the table here")
    args = ap.parse_args()
    files: list[Path] = []
    for pattern in args.logs:  # expand globs ourselves: PowerShell and cmd do not
        hits = sorted(Path().glob(pattern)) if any(c in pattern for c in '*?[') else [Path(pattern)]
        files += hits or [Path(pattern)]
    rows = [analyse(f) for f in files]
    keys: list[str] = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    short = [k for k in keys if k not in GAINS]
    width = {k: max(len(k), *(len(str(r.get(k, ""))) for r in rows)) for k in keys}
    print(" ".join(k.rjust(width[k]) for k in short))
    for r in rows:
        print(" ".join(str(r.get(k, "")).rjust(width[k]) for k in short))
    print("\ngains:")
    for r in rows:
        print(f"  {r['log']}: " + ", ".join(f"{g.replace('MC_', '').replace('MPC_', 'mpc ')} {r.get(g)}" for g in GAINS if r.get(g) is not None))
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            wtr = csv.DictWriter(fh, fieldnames=keys)
            wtr.writeheader()
            wtr.writerows(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
