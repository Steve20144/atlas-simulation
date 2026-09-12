import { useEffect } from "react";
import { useTiltlabStore } from "../store";

const btn = "rounded border border-slate-600 px-2 py-0.5 text-xs hover:bg-slate-700 disabled:opacity-40";
const POLL_MS = 3000;

/**
 * Launch Gazebo for the current scenario from the app: SITL (gz sim, no hardware) or HITL
 * (Gazebo Classic + the Pixhawk over USB). The backend exports the harness and starts
 * scripts/wsl/tiltlab_gazebo.sh in the matching WSL distro; this panel only shows the outcome
 * and the launcher's last console lines while a session runs.
 */
export default function GazeboPanel() {
  const gazebo = useTiltlabStore((s) => s.gazebo);
  const launch = useTiltlabStore((s) => s.launchGazebo);
  const poll = useTiltlabStore((s) => s.pollGazebo);
  const stop = useTiltlabStore((s) => s.stopGazebo);
  const reset = useTiltlabStore((s) => s.resetGazebo);

  useEffect(() => {
    if (!gazebo.running) return;
    const id = setInterval(() => void poll(), POLL_MS);
    return () => clearInterval(id);
  }, [gazebo.running, poll]);

  return (
    <div className="flex flex-col gap-1" data-testid="gazebo-panel">
      <h3 className="text-xs font-semibold">Launch Gazebo</h3>
      <div className="flex gap-2">
        <button
          className={btn}
          disabled={gazebo.running}
          title="gz sim in Ubuntu-24.04 with PX4 SITL; no hardware. Exports the harness first."
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
        <button
          className={btn}
          disabled={!gazebo.running}
          title="Disarm and put the model back where it spawned (Gazebo's own Reset Time only rewinds the clock). In HITL, a flight termination latched by a flip needs a board reboot: this does it and relaunches."
          onClick={() => void reset()}
        >
          Reset
        </button>
        <button className={btn} disabled={!gazebo.running} onClick={() => void stop()}>
          Stop
        </button>
        <span className="self-center text-[10px] text-slate-400">
          {gazebo.running ? `${gazebo.mode?.toUpperCase()} running` : "idle"}
        </span>
      </div>
      {gazebo.message && <p className="break-all text-[10px] text-slate-400">{gazebo.message}</p>}
      {gazebo.running && gazebo.tail.length > 0 && (
        <pre className="max-h-32 overflow-auto rounded bg-slate-900 p-1 text-[9px] leading-tight text-slate-300">
          {gazebo.tail.join("\n")}
        </pre>
      )}
    </div>
  );
}
