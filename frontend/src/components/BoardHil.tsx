import { useState } from "react";
import { useVectraStore } from "../store";

/**
 * SYS_HITL on the Pixhawk (0 off, 1 HITL, 2 SIH) and the reboot PX4 needs afterwards: rcS only
 * reads SYS_HITL at boot (line 324 of the pinned tree starts the sensors in HIL mode when it is
 * > 0). HIL off restores the whole HITL set from the flight backup, because the HIL airframe
 * (SYS_AUTOSTART 1001) runs `param set SYS_HITL 1` at every boot and a bare 0 comes back as 1.
 * Shown once a status check has read the value. Reboot asks for a second click.
 */
export default function BoardHil() {
  const status = useVectraStore((s) => s.board.status);
  const busy = useVectraStore((s) => s.board.pushing || s.board.checking);
  const gazeboRunning = useVectraStore((s) => s.gazebo.running);
  const { setBoardHitl, rebootBoard } = useVectraStore.getState();
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
        title="Back to flight: SYS_HITL 0 plus SYS_AUTOSTART, EKF2_EN, sensor presence, IMU calibration, GPS port and the HITL gains restored from the flight parameter backup (a bare SYS_HITL 0 is set back to 1 by the HIL airframe at boot). Current values are backed up under exports/board. Reboot afterwards."
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
