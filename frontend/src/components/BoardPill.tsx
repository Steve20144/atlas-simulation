import { useTiltlabStore } from "../store";

/**
 * The board status button in the top bar. One click asks the backend whether a Pixhawk is
 * heartbeating on USB (GET /api/board/status); the pill shows the result until the next check.
 */
export default function BoardPill() {
  const status = useTiltlabStore((s) => s.board.status);
  const checking = useTiltlabStore((s) => s.board.checking);
  const message = useTiltlabStore((s) => s.board.message);
  const checkBoard = useTiltlabStore((s) => s.checkBoard);

  const state = checking ? "checking" : status === null ? "unknown" : status.connected ? "connected" : "offline";
  const dot = { checking: "ui-dot-busy", unknown: "", connected: "ui-dot-ok", offline: "ui-dot-bad" }[state];
  const label = {
    checking: "checking",
    unknown: "board: check",
    connected: `board: ${status?.port ?? ""}${status?.firmware ? ` v${status.firmware}` : ""}`,
    offline: "board: offline",
  }[state];

  return (
    <button
      className="ui-btn"
      data-testid="board-pill"
      data-state={state}
      title={message || "Check whether the Pixhawk 6X Pro is connected over USB"}
      disabled={checking}
      onClick={() => void checkBoard()}
    >
      <span className={`ui-dot ${dot}`} />
      {label}
      {status?.connected && status.armed && (
        <span style={{ color: "var(--ui-warn)" }} title="the vehicle is armed">
          armed
        </span>
      )}
    </button>
  );
}
