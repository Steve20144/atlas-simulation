"""Hover quality report from a PX4 ulog: attitude, position, torque demand and gains.

    uv run --project backend python scripts/hover_report.py <file.ulg> [--window 20]

Prints, for the last ``window`` seconds (and the window before it): roll, pitch and yaw
peak-to-peak and dominant frequency, attitude setpoint swing (what the position loop asked
for), rate setpoint versus measured rate, normalised torque demand per axis (1 = full
authority, so values near 1 mean the allocator is saturating), position and altitude hold,
mean motor command, and the MC_/MPC_ gains the log ran with. Units: degrees, metres, seconds.

Reading the numbers: attitude swing with the *setpoint* swinging too is the position loop
driving the vehicle (slow the MPC_* loops); attitude swing with a steady setpoint and torque
demand near 1 is the rate loop ringing at its saturation limit (lower MC_*RATE_P, tighten the
integrator limit); gains far above the exported airframe means autotune ran (MC_AT_EN 0).
"""

from __future__ import annotations

import argparse

import numpy as np
from pyulog import ULog

GAINS = (
    "MC_ROLLRATE_P", "MC_PITCHRATE_P", "MC_YAWRATE_P", "MC_ROLL_P", "MC_PITCH_P", "MC_YAW_P",
    "MC_AT_EN", "MPC_XY_VEL_P_ACC", "MPC_XY_P", "MPC_Z_VEL_P_ACC", "MPC_TILTMAX_AIR", "THR_MDL_FAC",
)  # fmt: skip


def euler_deg(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))))
    return roll, pitch, yaw


def dominant_hz(x: np.ndarray, t: np.ndarray) -> float:
    if len(x) < 8:
        return 0.0
    x = x - x.mean()
    f = np.fft.rfftfreq(len(x), d=float(np.median(np.diff(t))))
    p = np.abs(np.fft.rfft(x))
    p[0] = 0
    return float(f[p.argmax()])


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("ulog")
    ap.add_argument("--window", type=float, default=20.0, help="seconds at the end of the log")
    args = ap.parse_args()
    u = ULog(args.ulog)
    t0 = u.start_timestamp / 1e6
    d = {x.name: x.data for x in u.data_list}

    att = d["vehicle_attitude"]
    t = att["timestamp"] / 1e6 - t0
    roll, pitch, yaw = euler_deg(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
    tend = float(t[-1])
    print(f"{args.ulog}: {tend:.0f} s of log")

    sp = d.get("vehicle_attitude_setpoint")
    rs, av = d.get("vehicle_rates_setpoint"), d.get("vehicle_angular_velocity")
    tq, lp, am = (
        d.get("vehicle_torque_setpoint"),
        d.get("vehicle_local_position"),
        d.get("actuator_motors"),
    )
    for lo, hi in ((tend - 2 * args.window, tend - args.window), (tend - args.window, tend)):
        if lo < 0:
            continue
        m = (t > lo) & (t < hi)
        print(f"\n{lo:.0f} to {hi:.0f} s")
        for name, sig in (("roll", roll), ("pitch", pitch), ("yaw", yaw)):
            print(
                f"  {name:5s} pk-pk {np.ptp(sig[m]):6.1f} deg  std {sig[m].std():5.2f}  @ {dominant_hz(sig[m], t[m]):.2f} Hz"
            )
        if sp is not None:
            ts = sp["timestamp"] / 1e6 - t0
            ms = (ts > lo) & (ts < hi)
            r_sp, p_sp, _ = euler_deg(*(sp[f"q_d[{i}]"][ms] for i in range(4)))
            print(
                f"  attitude setpoint swing: roll {np.ptp(r_sp):.1f} pitch {np.ptp(p_sp):.1f} deg"
            )
        if rs is not None and av is not None:
            tr, ta = rs["timestamp"] / 1e6 - t0, av["timestamp"] / 1e6 - t0
            mr, ma = (tr > lo) & (tr < hi), (ta > lo) & (ta < hi)
            for i, name in enumerate(("roll", "pitch", "yaw")):
                print(
                    f"  {name:5s} rate sp pk-pk {np.degrees(np.ptp(rs[name][mr])):5.0f} deg/s, measured {np.degrees(np.ptp(av[f'xyz[{i}]'][ma])):5.0f} deg/s"
                )
        if tq is not None:
            tt = tq["timestamp"] / 1e6 - t0
            mt = (tt > lo) & (tt < hi)
            print(
                "  torque demand pk-pk (1 = full authority): "
                + " ".join(
                    f"{n} {np.ptp(tq[f'xyz[{i}]'][mt]):.2f}"
                    for i, n in enumerate(("roll", "pitch", "yaw"))
                )
            )
        if lp is not None:
            tl = lp["timestamp"] / 1e6 - t0
            ml = (tl > lo) & (tl < hi)
            print(
                f"  position pk-pk x {np.ptp(lp['x'][ml]):.2f} y {np.ptp(lp['y'][ml]):.2f} z {np.ptp(lp['z'][ml]):.2f} m, altitude {-lp['z'][ml].mean():.2f} m"
            )
        if am is not None:
            tm = am["timestamp"] / 1e6 - t0
            mm = (tm > lo) & (tm < hi)
            cmds = [
                float(np.nanmean(am[f"control[{i}]"][mm]))
                for i in range(12)
                if f"control[{i}]" in am
            ]
            cmds = [c for c in cmds if np.isfinite(c)]
            print(f"  motor command mean {np.mean(cmds):.3f}, max {np.max(cmds):.3f}")
    p = u.initial_parameters
    print("\ngains in this log:", {k: round(float(p[k]), 3) for k in GAINS if k in p})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
