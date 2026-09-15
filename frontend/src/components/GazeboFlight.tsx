import { useEffect } from "react";
import { useVectraStore } from "../store";

const btn = "ui-btn";
const PROBE_MS = 3000;

/** Flight controls for a running SITL session: take off, land, pull the newest log, and a live
 * probe (armed, mode, height, allocator setpoints achieved). Shown only while a sim runs. */
export default function GazeboFlight() {
  const gazebo = useVectraStore((s) => s.gazebo);
  const probe = useVectraStore((s) => s.gazeboProbe);
  const takeoff = useVectraStore((s) => s.takeoffGazebo);
  const land = useVectraStore((s) => s.landGazebo);
  const pullLog = useVectraStore((s) => s.pullGazeboLog);
  const refresh = useVectraStore((s) => s.probeGazebo);

  useEffect(() => {
    if (!gazebo.ready) return;
    void refresh();
    const id = setInterval(() => void refresh(), PROBE_MS);
    return () => clearInterval(id);
  }, [gazebo.ready, refresh]);

  const sitl = gazebo.mode === "sitl";
  const allocatorBad = probe?.torque_achieved === false || probe?.thrust_achieved === false;
  const wedged = probe?.arming_topic_age_s != null && probe.arming_topic_age_s > 120;

  return (
    <div className="flex flex-col gap-1 rounded border border-slate-700 bg-slate-950/60 p-2" data-testid="gazebo-flight">
      <div className="flex flex-wrap items-center gap-2">
        <button
          className={btn}
          disabled={!gazebo.ready || !sitl || probe?.armed === true}
          title="commander takeoff: arms, climbs to MIS_TAKEOFF_ALT (2.5 m), holds position with a fixed heading"
          onClick={() => void takeoff()}
        >
          Take off
        </button>
        <button className={btn} disabled={!gazebo.ready || !sitl} onClick={() => void land()}>
          Land
        </button>
        <button
          className={btn}
          disabled={!gazebo.ready || !sitl}
          title="Copy the newest SITL ulog into exports/logs as sitl_<name>.ulg"
          onClick={() => void pullLog()}
        >
          Pull log
        </button>
        {!sitl && <span className="text-[10px] text-slate-500">HITL flies from the transmitter</span>}
      </div>
      {probe && (
        <p className="text-[10px] tabular-nums text-slate-300" data-testid="gazebo-probe">
          {probe.ok
            ? `${probe.armed ? "armed" : "disarmed"} · ${probe.nav_mode ?? "?"} · ${probe.height_m ?? "?"} m` +
              (probe.climb_mps != null ? ` (${probe.climb_mps > 0 ? "+" : ""}${probe.climb_mps} m/s)` : "") +
              ` · allocator ${allocatorBad ? "NOT meeting setpoints" : "ok"}`
            : "no vehicle data yet"}
        </p>
      )}
      {allocatorBad && (
        <p className="text-[10px] text-rose-300">
          Torque or thrust setpoint not achieved: PX4 freezes the rate integrators in that direction. Fix the geometry the allocator sees (pid-tuning skill, section 3) before tuning gains.
        </p>
      )}
      {wedged && (
        <p className="text-[10px] text-rose-300">
          The arming topic is {Math.round(probe!.arming_topic_age_s!)} s old: the commander has stopped. Stop and launch again; Reset cannot recover it.
        </p>
      )}
    </div>
  );
}
