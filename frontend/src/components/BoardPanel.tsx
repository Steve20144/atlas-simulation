import { useState } from "react";
import { useVectraStore } from "../store";
import BoardHil from "./BoardHil";
import BoardMessages from "./BoardMessages";
import ThrustFeed from "./ThrustFeed";

/**
 * Pixhawk 6X Pro over USB. Check reports the heartbeat, firmware and flags; Upload writes the
 * previewed CA_* lines to the board through its NSH shell (integers stay integers), saves them
 * to flash, reads every value back and keeps a timestamped backup of the previous values in
 * exports/board/. Two clicks are needed: the second confirms the port and the parameter count.
 */
export default function BoardPanel() {
  const board = useVectraStore((s) => s.board);
  const lines = useVectraStore((s) => s.paramsLines);
  const gazeboHitl = useVectraStore((s) => s.gazebo.running && s.gazebo.mode === "hitl");
  const feeding = useVectraStore((s) => s.thrust.running);
  const { checkBoard, pushBoard, pullBoardLog, startThrustFeed, setBoardPort } = useVectraStore.getState();
  const [armed, setArmed] = useState(false);
  const st = board.status;
  const ports = st?.ports ?? [];
  const busy = board.checking || board.pushing || board.pulling || feeding;
  const canPush = lines.length > 0 && !busy && !gazeboHitl;
  const dot = board.checking ? "ui-dot-busy" : st === null ? "" : st.connected ? "ui-dot-ok" : "ui-dot-bad";

  const upload = () => {
    if (!armed) {
      setArmed(true);
      return;
    }
    setArmed(false);
    void pushBoard();
  };

  return (
    <div className="flex flex-col gap-2" data-testid="board-panel">
      <div className="flex items-center justify-between">
        <h3>Pixhawk 6X Pro</h3>
        <span className="ui-label flex items-center gap-2">
          <span className={`ui-dot ${dot}`} />
          {board.checking ? "checking" : st === null ? "not checked" : st.connected ? "connected" : "offline"}
        </span>
      </div>

      <div className="flex flex-wrap items-center gap-2">
        <select
          aria-label="Serial port"
          value={board.port}
          onChange={(e) => setBoardPort(e.target.value)}
          title="auto picks the first port with a Pixhawk USB id or name"
        >
          <option value="auto">auto</option>
          {ports.map((p) => (
            <option key={p.device} value={p.device}>
              {p.device}
              {p.pixhawk ? " (Pixhawk)" : ""}
            </option>
          ))}
          {board.port !== "auto" && !ports.some((p) => p.device === board.port) && (
            <option value={board.port}>{board.port}</option>
          )}
        </select>
        <button className="ui-btn" disabled={busy} onClick={() => void checkBoard()}>
          Check
        </button>
        <button
          className="ui-btn"
          disabled={busy || gazeboHitl}
          title={
            gazeboHitl
              ? "the HITL session owns the serial port; stop it first"
              : "Download the newest .ulg the board recorded (highest GPS time, then highest id) into exports/logs/. About 50 kB/s over USB, so a long flight takes minutes."
          }
          onClick={() => void pullBoardLog()}
        >
          {board.pulling ? "downloading log" : "Latest flight data"}
        </button>
        <button
          className="ui-btn"
          disabled={busy || gazeboHitl}
          title="Live view of what the board makes of the throttle stick: RC pulse, PX4 throttle, thrust setpoint and the pulse width on every ESC output, 10 Hz. The outputs stay live: fans and ESCs unpowered."
          onClick={() => void startThrustFeed()}
        >
          Test thrust
        </button>
        <button
          className={armed ? "ui-btn-primary ui-btn" : "ui-btn"}
          disabled={!canPush}
          title={
            gazeboHitl
              ? "the HITL session owns the serial port; stop it first"
              : "Write the previewed parameters to the board, save and verify. Fans and ESCs unpowered."
          }
          onClick={upload}
        >
          {board.pushing ? "uploading" : armed ? `confirm: write ${lines.length} params` : "Upload params"}
        </button>
        {armed && (
          <button className="ui-btn" onClick={() => setArmed(false)}>
            cancel
          </button>
        )}
      </div>

      {st?.connected && (
        <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-0.5 text-[11px]">
          <dt className="ui-label self-center">port</dt>
          <dd className="tabular-nums">{st.port}</dd>
          <dt className="ui-label self-center">firmware</dt>
          <dd className="tabular-nums">{st.firmware ?? "unknown"}</dd>
          <dt className="ui-label self-center">mode</dt>
          <dd className="tabular-nums">
            {st.mode ?? "?"}
            {st.hil ? " / HIL" : ""}
            {st.armed ? " / ARMED" : ""}
          </dd>
        </dl>
      )}

      <BoardHil />
      <ThrustFeed />
      <BoardMessages />
    </div>
  );
}
