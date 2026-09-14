"""Triage a PX4 ulog from the Pixhawk: what the pilot asked, what the controller wanted, what
the motors got, and which known failure signatures are present.

    uv run --project backend python scripts/flight_debug.py <file.ulg> [--json out.json]

Sections (all read from the log, nothing from the board):
  1. log: duration, PX4 commit, airframe, hover pitch offset (SENS_BOARD_Y_OFF), CA_* summary
  2. events: every logged message at warning level or above, plus arming and disarming reasons
  3. arming: armed windows, nav (flight) modes seen, HIL and RC-calibration flags
  4. stick -> thrust: raw throttle channel, PX4 throttle (manual_control_setpoint.throttle),
     collective thrust setpoint (vehicle_thrust_setpoint z), the fitted slope stick -> thrust
  5. motors: per-motor min / mean / max of actuator_motors while armed, share of samples at the
     limits, output pulse widths (actuator_outputs), allocator saturation and unallocated torque
  6. attitude: roll / pitch mean and swing while armed, mean pitch on the ground
  7. findings: heuristics for the failure signatures seen on this aircraft (see SKILL.md)

Units: seconds, degrees, microseconds for pulse widths, normalised 0..1 for commands. Frame FRD.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
from pyulog import ULog

ARM_REASONS = {
    0: "transition_to_standby", 1: "stick_gesture", 2: "rc_switch", 3: "command_internal",
    4: "command_external", 5: "mission_start", 6: "safety_button", 7: "auto_disarm_land",
    8: "auto_disarm_preflight", 9: "kill_switch", 10: "lockdown", 11: "failure_detector",
    12: "shutdown", 13: "unit_test", 14: "rc_button", 15: "failsafe", 16: "auto_preflight_disarming",
}  # msg/versioned/VehicleStatus.msg ARM_DISARM_REASON_*
NAV_STATES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
    10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF",
    18: "AUTO_LAND", 19: "AUTO_FOLLOW_TARGET", 20: "AUTO_PRECLAND", 21: "ORBIT",
}  # msg/versioned/VehicleStatus.msg NAVIGATION_STATE_*
PARAMS_OF_INTEREST = (
    "SYS_AUTOSTART", "SYS_HITL", "CA_AIRFRAME", "CA_METHOD", "CA_ROTOR_COUNT", "MC_AIRMODE",
    "SENS_BOARD_ROT", "SENS_BOARD_Y_OFF", "MPC_THR_HOVER", "MPC_MANTHR_MIN", "MPC_THR_MIN",
    "MPC_THR_MAX", "THR_MDL_FAC", "COM_SPOOLUP_TIME", "RC_MAP_THROTTLE", "RC_MAP_ARM_SW",
    "RC3_MIN", "RC3_MAX", "RC3_TRIM", "RC3_DZ", "COM_RC_IN_MODE", "CBRK_SUPPLY_CHK", "EKF2_EN",
    "MC_ROLLRATE_P", "MC_PITCHRATE_P", "MC_YAWRATE_P", "MC_ROLL_P", "MC_PITCH_P",
)
LOG_LEVEL_WARNING = 52  # ulog levels follow syslog: '4' = warning, '3' error, ... as ASCII


def topic(u: ULog, name: str, instance: int = 0):
    for d in u.data_list:
        if d.name == name and d.multi_id == instance:
            return d
    return None


def col(d, key: str) -> np.ndarray:
    return np.asarray(d.data[key], dtype=float)


def t_s(d, t0: int) -> np.ndarray:
    return (np.asarray(d.data["timestamp"], dtype=float) - t0) / 1e6


def euler_deg(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    return roll, pitch


def armed_windows(u: ULog, t0: int) -> list[tuple[float, float]]:
    """[start, end] seconds where actuator_armed.armed was true."""
    d = topic(u, "actuator_armed")
    if d is None:
        return []
    t, a = t_s(d, t0), col(d, "armed") > 0.5
    out: list[tuple[float, float]] = []
    start = None
    for ti, ai in zip(t, a, strict=True):
        if ai and start is None:
            start = ti
        elif not ai and start is not None:
            out.append((start, ti))
            start = None
    if start is not None:
        out.append((start, float(t[-1])))
    return out


def in_windows(t: np.ndarray, windows: list[tuple[float, float]]) -> np.ndarray:
    m = np.zeros(len(t), dtype=bool)
    for a, b in windows:
        m |= (t >= a) & (t <= b)
    return m


def analyse(path: Path) -> dict[str, Any]:
    u = ULog(str(path))
    t0 = u.start_timestamp
    dur = (u.last_timestamp - t0) / 1e6
    p = u.initial_parameters
    rep: dict[str, Any] = {"file": str(path), "duration_s": round(dur, 1)}
    rep["log"] = {
        "px4_commit": u.msg_info_dict.get("ver_sw", ""),
        "params": {k: p.get(k) for k in PARAMS_OF_INTEREST if k in p},
        "rotor_km_nonzero": sum(1 for i in range(12) if p.get(f"CA_ROTOR{i}_KM", 0) != 0),
    }

    # ---- events
    msgs = [
        {"t": round((m.timestamp - t0) / 1e6, 2), "level": m.log_level, "text": m.message.strip()}
        for m in u.logged_messages
    ]
    rep["events"] = {
        "warnings_and_errors": [m for m in msgs if m["level"] <= LOG_LEVEL_WARNING],
        "arming_messages": [m for m in msgs if "rmed" in m["text"] or "arm" in m["text"].lower()][:20],
    }

    # ---- arming / modes / flags
    vs = topic(u, "vehicle_status")
    windows = armed_windows(u, t0)
    rep["arming"] = {"armed_windows_s": [(round(float(a), 1), round(float(b), 1)) for a, b in windows]}
    if vs is not None:
        nav = col(vs, "nav_state").astype(int)
        rep["arming"].update(
            {
                "nav_states": sorted({NAV_STATES.get(int(n), str(n)) for n in nav}),
                "arming_reasons": sorted(
                    {ARM_REASONS.get(int(r), str(r)) for r in col(vs, "latest_arming_reason")}
                ),
                "disarming_reasons": sorted(
                    {ARM_REASONS.get(int(r), str(r)) for r in col(vs, "latest_disarming_reason")}
                ),
                "hil_state_max": int(col(vs, "hil_state").max()),
                "rc_calibration_in_progress_ever": bool(
                    "rc_calibration_in_progress" in vs.data
                    and col(vs, "rc_calibration_in_progress").max() > 0
                ),
                "failsafe_ever": bool(col(vs, "failsafe").max() > 0) if "failsafe" in vs.data else None,
            }
        )
    aa = topic(u, "actuator_armed")
    if aa is not None:
        rep["arming"]["lockdown_ever"] = bool(col(aa, "lockdown").max() > 0)
        rep["arming"]["kill_ever"] = bool(col(aa, "kill").max() > 0)

    # ---- stick -> thrust
    st: dict[str, Any] = {}
    rc = topic(u, "input_rc")
    ch = int(p.get("RC_MAP_THROTTLE", 3))
    if rc is not None and f"values[{ch - 1}]" in rc.data:
        raw = col(rc, f"values[{ch - 1}]")
        st["rc_throttle_us"] = {"min": int(raw.min()), "max": int(raw.max()), "span": int(raw.max() - raw.min())}
        st["rc_lost_ever"] = bool(col(rc, "rc_lost").max() > 0) if "rc_lost" in rc.data else None
    mc = topic(u, "manual_control_setpoint")
    if mc is not None:
        thr = col(mc, "throttle")
        st["px4_throttle_0_1"] = {  # manual_control_setpoint.throttle is -1..1; 0..1 here
            "min": round(float((thr.min() + 1) / 2), 3), "max": round(float((thr.max() + 1) / 2), 3)
        }
    ts = topic(u, "vehicle_thrust_setpoint")
    if ts is not None:
        tz = -col(ts, "xyz[2]")  # FRD: negative z is up, so -z is the collective 0..1
        tt = t_s(ts, t0)
        m = in_windows(tt, windows) if windows else np.ones(len(tt), dtype=bool)
        if m.any():
            st["thrust_sp_armed"] = {
                "min": round(float(tz[m].min()), 3), "mean": round(float(tz[m].mean()), 3),
                "max": round(float(tz[m].max()), 3),
            }
        if mc is not None and m.any():
            # resample throttle onto thrust timestamps, fit thrust = a * throttle + b (low stick)
            thr01 = (thr + 1) / 2
            thr_i = np.interp(tt, t_s(mc, t0), thr01)
            low = m & (thr_i < 0.4)
            if low.sum() > 10 and np.ptp(thr_i[low]) > 0.02:
                a, b = np.polyfit(thr_i[low], tz[low], 1)
                st["thrust_per_stick_below_40pct"] = round(float(a), 2)
                st["thrust_at_zero_stick"] = round(float(b), 3)
    rep["stick_to_thrust"] = st

    # ---- motors / outputs / allocator
    mo: dict[str, Any] = {}
    am = topic(u, "actuator_motors")
    n_rot = int(p.get("CA_ROTOR_COUNT", 10))
    if am is not None:
        ta = t_s(am, t0)
        m = in_windows(ta, windows) if windows else np.ones(len(ta), dtype=bool)
        if m.any():
            per = []
            for i in range(n_rot):
                key = f"control[{i}]"
                if key not in am.data:
                    break
                c = col(am, key)[m]
                c = c[np.isfinite(c)]
                if len(c) == 0:
                    per.append({"motor": i, "nan": True})
                    continue
                per.append(
                    {
                        "motor": i, "min": round(float(c.min()), 3), "mean": round(float(c.mean()), 3),
                        "max": round(float(c.max()), 3), "at_zero_pct": round(100 * float((c < 0.01).mean()), 1),
                        "at_full_pct": round(100 * float((c > 0.99).mean()), 1),
                    }
                )
            mo["actuator_motors_armed"] = per
            mo["all_motors_zero_while_armed_pct"] = round(
                100 * float(np.all([col(am, f"control[{i}]")[m] < 0.01 for i in range(len(per))], axis=0).mean()), 1
            ) if per else None
    ao = topic(u, "actuator_outputs")
    if ao is not None:
        outs = []
        for inst in (0, 1, 2):
            d = topic(u, "actuator_outputs", inst)
            if d is None:
                continue
            n = int(col(d, "noutputs").max()) if "noutputs" in d.data else 8
            tt = t_s(d, t0)
            m = in_windows(tt, windows) if windows else np.ones(len(tt), dtype=bool)
            if not m.any():
                continue
            outs.append(
                {
                    "instance": inst,
                    "pwm_us_max_per_output": [int(col(d, f"output[{i}]")[m].max()) for i in range(min(n, 16))],
                }
            )
        mo["actuator_outputs_armed"] = outs
    cas = topic(u, "control_allocator_status")
    if cas is not None:
        tc = t_s(cas, t0)
        m = in_windows(tc, windows) if windows else np.ones(len(tc), dtype=bool)
        if m.any():
            sat = np.stack([col(cas, f"actuator_saturation[{i}]")[m] for i in range(n_rot)])
            mo["allocator_armed"] = {
                "torque_achieved_pct": round(100 * float(col(cas, "torque_setpoint_achieved")[m].mean()), 1),
                "thrust_achieved_pct": round(100 * float(col(cas, "thrust_setpoint_achieved")[m].mean()), 1),
                "unallocated_torque_abs_max": [
                    round(float(np.abs(col(cas, f"unallocated_torque[{i}]")[m]).max()), 3) for i in range(3)
                ],
                "motor_saturation_upper_pct": round(100 * float((sat == 2).any(axis=0).mean()), 1),
                "motor_saturation_lower_pct": round(100 * float((sat == -2).any(axis=0).mean()), 1),
            }
    rep["motors"] = mo

    # ---- attitude
    att = topic(u, "vehicle_attitude")
    at: dict[str, Any] = {}
    if att is not None:
        roll, pitch = euler_deg(*(col(att, f"q[{i}]") for i in range(4)))
        tt = t_s(att, t0)
        m = in_windows(tt, windows) if windows else np.zeros(len(tt), dtype=bool)
        at["pitch_deg_disarmed_mean"] = round(float(pitch[~m].mean()), 1) if (~m).any() else None
        if m.any():
            at["armed"] = {
                "roll_mean": round(float(roll[m].mean()), 1), "roll_p2p": round(float(np.ptp(roll[m])), 1),
                "pitch_mean": round(float(pitch[m].mean()), 1), "pitch_p2p": round(float(np.ptp(pitch[m])), 1),
            }
    rep["attitude"] = at

    rep["findings"] = findings(rep)
    return rep


def findings(rep: dict[str, Any]) -> list[str]:
    """Known failure signatures of this aircraft, in the order to check them."""
    out: list[str] = []
    prm = rep["log"]["params"]
    arm = rep.get("arming", {})
    if prm.get("SYS_HITL", 0) or arm.get("hil_state_max"):
        out.append("HITL: the board ran with SYS_HITL set, real sensors and outputs were off. Use HIL off in Vectra, reboot.")
    if int(prm.get("SYS_AUTOSTART", 0)) == 1001:
        out.append("SYS_AUTOSTART 1001 is PX4's HIL airframe: it re-sets SYS_HITL 1 at every boot. HIL off in Vectra restores the flight airframe.")
    if arm.get("rc_calibration_in_progress_ever"):
        out.append("rc_calibration_in_progress was set: PX4 ignores the sticks and refuses arming until a reboot (QGC Radio calibration left open).")
    if arm.get("lockdown_ever"):
        out.append("actuator_armed.lockdown was set: outputs held at disarmed values (auto-disarm follows after 5 s outside HITL).")
    if arm.get("kill_ever"):
        out.append("kill switch engaged during the log: outputs at disarmed values while armed.")
    if not arm.get("armed_windows_s"):
        out.append("never armed in this log: look at the warnings above (preflight) and at the arming reasons.")
    if any(r in ("auto_disarm_preflight", "auto_preflight_disarming") for r in arm.get("disarming_reasons", [])):
        out.append("auto disarm before takeoff: the board sat armed without throttle for COM_DISARM_PRFLT seconds.")
    mo = rep.get("motors", {})
    z = mo.get("all_motors_zero_while_armed_pct")
    if z is not None and z > 50:
        out.append(f"all motors at 0 for {z}% of the armed time: the allocator produced no output for this geometry (infeasible CA_ROTOR set; check the scenario hover trim in Vectra, push a geometry marked exact).")
    dead = [m["motor"] for m in mo.get("actuator_motors_armed", []) if not m.get("nan") and m["at_zero_pct"] >= 95]
    thrust_asked = (rep.get("stick_to_thrust", {}).get("thrust_sp_armed") or {}).get("max", 0) > 0.05
    if dead and thrust_asked and len(dead) < len(mo.get("actuator_motors_armed", [])):
        out.append(f"motors {', '.join(str(i) for i in dead)} never left 0 while thrust was commanded: the allocator gives them nothing with this CA_ROTOR geometry (or they are not in the mix). Run the geometry through Vectra's stock hover.")
    if any(m.get("nan") for m in mo.get("actuator_motors_armed", [])):
        out.append("actuator_motors carried NaN: the mixing matrix could not be normalised (a torque axis with no authority).")
    al = mo.get("allocator_armed")
    if al and al["torque_achieved_pct"] < 50:
        out.append(f"torque setpoint achieved only {al['torque_achieved_pct']}% of the time while armed: the geometry lacks authority or motors sit at their limits.")
    if al and al["motor_saturation_upper_pct"] > 30:
        out.append(f"a motor was at its upper limit {al['motor_saturation_upper_pct']}% of the armed time: not enough thrust margin.")
    st = rep.get("stick_to_thrust", {})
    slope = st.get("thrust_per_stick_below_40pct")
    if slope is not None and slope > 1.5:
        out.append(f"thrust setpoint rises {slope:.1f}x faster than the stick below 40%: MPC_THR_HOVER {prm.get('MPC_THR_HOVER')} rescales the stick so mid-stick equals hover thrust; lower it or fly Acro/Manual for a bench test.")
    if st.get("thrust_at_zero_stick", 0) and st["thrust_at_zero_stick"] > 0.15:
        out.append(f"thrust setpoint is {st['thrust_at_zero_stick']:.2f} with the stick at the bottom: MPC_MANTHR_MIN / MPC_THR_MIN or the RC3 calibration.")
    rcu = st.get("rc_throttle_us")
    if rcu and rcu["span"] < 100 and arm.get("armed_windows_s"):
        out.append("the throttle channel barely moved (span < 100 us): transmitter model or channel mapping, not PX4.")
    if st.get("rc_lost_ever"):
        out.append("RC was lost at some point in the log.")
    at = rep.get("attitude", {})
    off = prm.get("SENS_BOARD_Y_OFF", 0)
    if at.get("pitch_deg_disarmed_mean") is not None and abs(off) > 5:
        out.append(f"SENS_BOARD_Y_OFF {off} deg: the board reads {at['pitch_deg_disarmed_mean']} deg pitch at rest, so Stabilized fights that offset on the ground (front fans up, rear down).")
    if rep["log"]["rotor_km_nonzero"] == 0:
        out.append("all CA_ROTORn_KM are 0: yaw comes only from tilted axes (fine for this geometry, but QGC's Actuators page shows no CW/CCW directions).")
    return out or ["no known signature matched; read the sections above against the symptom"]


def render(rep: dict[str, Any]) -> str:
    lines = [f"# flight_debug: {rep['file']} ({rep['duration_s']} s)", ""]
    lines.append("## log")
    lines.append(f"px4 {rep['log']['px4_commit'][:10]}  params: " + ", ".join(f"{k}={v:g}" if isinstance(v, float) else f"{k}={v}" for k, v in rep["log"]["params"].items()))
    lines.append(f"rotors with KM != 0: {rep['log']['rotor_km_nonzero']}")
    lines.append("\n## events (warning and above)")
    lines += [f"  {m['t']:7.2f}s  {m['text']}" for m in rep["events"]["warnings_and_errors"]] or ["  none"]
    lines.append("## arming messages")
    lines += [f"  {m['t']:7.2f}s  {m['text']}" for m in rep["events"]["arming_messages"]] or ["  none"]
    lines.append("\n## arming")
    lines += [f"  {k}: {v}" for k, v in rep["arming"].items()]
    lines.append("\n## stick -> thrust")
    lines += [f"  {k}: {v}" for k, v in rep["stick_to_thrust"].items()] or ["  no RC / thrust topics"]
    lines.append("\n## motors")
    for k, v in rep["motors"].items():
        if k == "actuator_motors_armed":
            lines.append("  actuator_motors while armed (0..1):")
            lines += [f"    m{m['motor']}: " + ("NaN" if m.get("nan") else f"min {m['min']} mean {m['mean']} max {m['max']} | at 0: {m['at_zero_pct']}%  at 1: {m['at_full_pct']}%") for m in v]
        else:
            lines.append(f"  {k}: {v}")
    lines.append("\n## attitude")
    lines += [f"  {k}: {v}" for k, v in rep["attitude"].items()] or ["  no attitude topic"]
    lines.append("\n## findings")
    lines += [f"  - {f}" for f in rep["findings"]]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog", type=Path)
    ap.add_argument("--json", type=Path, default=None, help="also write the full report as JSON")
    args = ap.parse_args()
    rep = analyse(args.ulog)
    print(render(rep))
    if args.json:
        args.json.write_text(json.dumps(rep, indent=2), encoding="utf-8")
        print(f"\njson: {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
