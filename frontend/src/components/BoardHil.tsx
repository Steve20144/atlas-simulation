import { useState } from "react";
import { useTiltlabStore } from "../store";

/**
 * SYS_HITL on the Pixhawk (0 off, 1 HITL, 2 SIH) and the reboot PX4 needs afterwards: rcS only
 * reads SYS_HITL at boot (line 324 of the pinned tree starts the sensors in HIL mode when it is
 * > 0). Shown once a status check has read the value. Reboot asks for a second click.
 */
export default function BoardHil() {
  const status = useTiltlabStore((s) => s.board.status);
  const busy = useTiltlabStore((s) => s.board.pushing || s.board.checking);
  const gazeboRunning = useTiltlabStore((s) => s.gazebo.running);
  const { setBoardHitl, rebootBoard } = useTiltlabStore.getState();
  const [armed, setArmed] = useState(false);
  if (!status?.connected) return null;
  const hitl = status.sys_hitl;
  const label = hitl === null ? "unread" : hitl === 0 ? "off" : hitl === 1 ? "on (HITL)" : `SIH (${hitl})`;

  const reboot = () => {
    if (!armed) {
      setArmed(true);
      return;
    }
    setArmed(false);
    void rebootBoard();
  };

  return (
    <div className="flex flex-wrap items-center gap-2" data-testid="board-hil">
      <span className="ui-label">SYS_HITL</span>
      <span className="flex items-center gap-1.5 tabular-nums text-[11px]">
        <span className={`ui-dot ${hitl === null ? "" : hitl > 0 ? "ui-dot-warn" : "ui-dot-ok"}`} />
        {label}
      </span>
      <button
        className={hitl === 1 ? "ui-btn-active" : "ui-btn"}
        aria-pressed={hitl === 1}
        disabled={busy || hitl === 1}
        title="SYS_HITL 1: PX4 boots with simulated sensors and pwm_out_sim (needs the HITL firmware build). Fans and ESCs unpowered."
        onClick={() => void setBoardHitl(true)}
      >
        HIL on
      </button>
      <button
        className={hitl === 0 ? "ui-btn-active" : "ui-btn"}
        aria-pressed={hitl === 0}
        disabled={busy || hitl === 0}
        title="SYS_HITL 0: normal flight firmware behaviour on the next boot"
        onClick={() => void setBoardHitl(false)}
      >
        HIL off
      </button>
      <button
        className={armed ? "ui-btn ui-btn-primary" : "ui-btn"}
        disabled={busy || gazeboRunning}
        title={
          gazeboRunning
            ? "stop the Gazebo session first; it owns the link"
            : "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: the board restarts and the USB link is back in a few seconds"
        }
        onClick={reboot}
      >
        {armed ? "confirm reboot" : "Reboot board"}
      </button>
      {armed && (
        <button className="ui-btn" onClick={() => setArmed(false)}>
          cancel
        </button>
      )}
    </div>
  );
}
