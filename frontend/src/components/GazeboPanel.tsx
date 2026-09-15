import { useEffect } from "react";
import { useVectraStore } from "../store";
import GazeboFlight from "./GazeboFlight";

const btn = "ui-btn";
const POLL_MS = 3000;

/**
 * Launch Gazebo for the current scenario from the app: SITL (gz sim, no hardware) or HITL
 * (Gazebo Classic + the Pixhawk over USB). The backend exports the harness, stops whatever sim
 * is up in that distro and starts scripts/wsl/vectra_gazebo.sh; this panel shows readiness,
 * the launcher's last console lines and, once PX4 is ready, the flight controls.
 */
export default function GazeboPanel() {
  const gazebo = useVectraStore((s) => s.gazebo);
  const headless = useVectraStore((s) => s.gazeboHeadless);
  const setHeadless = useVectraStore((s) => s.setGazeboHeadless);
  const launch = useVectraStore((s) => s.launchGazebo);
  const poll = useVectraStore((s) => s.pollGazebo);
  const stop = useVectraStore((s) => s.stopGazebo);
  const reset = useVectraStore((s) => s.resetGazebo);
  const wslShutdown = useVectraStore((s) => s.wslShutdown);

  useEffect(() => {
    if (!gazebo.running) return;
    const id = setInterval(() => void poll(), POLL_MS);
    return () => clearInterval(id);
  }, [gazebo.running, poll]);

  const state = !gazebo.running
    ? "idle"
    : gazebo.ready
      ? `${gazebo.mode?.toUpperCase()} ready`
      : `${gazebo.mode?.toUpperCase()} starting${gazebo.uptime_s ? ` ${Math.round(gazebo.uptime_s)} s` : ""}`;

  return (
    <div className="flex flex-col gap-1" data-testid="gazebo-panel">
      <h3>Launch Gazebo</h3>
      <div className="flex flex-wrap items-center gap-2">
        <button
          className={btn}
          disabled={gazebo.running}
          title="gz sim in Ubuntu-24.04 with PX4 SITL; no hardware. Exports the harness, stops any stale sim, starts fresh."
          onClick={() => void launch("sitl")}
        >
          SITL
        </button>
        <button
          className={btn}
          disabled={gazebo.running}
          title="Gazebo Classic in Ubuntu-22.04 driving the Pixhawk over USB. Needs the HITL firmware, the exported params on the board and the USB attached to WSL (docs/hitl_runbook.md). Fans and ESCs unpowered."
          onClick={() => void launch("hitl")}
        >
          HITL
        </button>
        <label className="flex items-center gap-1 text-[10px] text-slate-400" title="No gz GUI window: faster, and immune to WSLg display faults. QGroundControl still connects.">
          <input type="checkbox" checked={headless} onChange={(e) => setHeadless(e.target.checked)} />
          headless
        </label>
        <button
          className={btn}
          disabled={!gazebo.running}
          title="Restart PX4 and put the model back where it spawned (fresh EKF, integrators and log). In HITL, a flight termination latched by a flip needs a board reboot: this does it and relaunches."
          onClick={() => void reset()}
        >
          Reset
        </button>
        <button className={btn} disabled={!gazebo.running} onClick={() => void stop()}>
          Stop
        </button>
        <span className="text-[10px] text-slate-400" data-testid="gazebo-state">{state}</span>
      </div>
      {gazebo.wslg_copy_mode && (
        <div className="flex items-center gap-2 rounded border border-amber-600 bg-amber-900/40 px-2 py-1 text-[10px] text-amber-200">
          WSLg is in copy mode: the Gazebo window will be grey. Restart WSL (stops both distros), then launch again.
          <button className={btn} onClick={() => void wslShutdown()}>
            restart WSL
          </button>
        </div>
      )}
      {gazebo.message && <p className="break-all text-[10px] text-slate-400">{gazebo.message}</p>}
      {gazebo.running && <GazeboFlight />}
      {gazebo.running && gazebo.tail.length > 0 && (
        <pre className="max-h-32 overflow-auto rounded bg-slate-900 p-1 text-[9px] leading-tight text-slate-300">
          {gazebo.tail.join("\n")}
        </pre>
      )}
    </div>
  );
}
