import { useVectraStore } from "../store";

/** Status line, pulled log link, backup path and read-back mismatches of the Pixhawk card. */
export default function BoardMessages() {
  const board = useVectraStore((s) => s.board);
  return (
    <>
      {board.message && (
        <p
          className="break-all text-[10px]"
          style={{ color: board.result && board.result.mismatches.length > 0 ? "var(--ui-bad)" : "var(--ui-muted)" }}
        >
          {board.message}
        </p>
      )}
      {board.log && (
        <p className="break-all text-[10px]" style={{ color: "var(--ui-dim)" }}>
          saved {board.log.path}{" "}
          <a className="underline" href={board.log.url} download>
            download .ulg
          </a>
        </p>
      )}
      {board.result?.backup && (
        <p className="break-all text-[10px]" style={{ color: "var(--ui-dim)" }}>
          backup {board.result.backup}
        </p>
      )}
      {board.result && board.result.mismatches.length > 0 && (
        <ul className="text-[10px] tabular-nums" style={{ color: "var(--ui-bad)" }}>
          {board.result.mismatches.slice(0, 8).map((m) => (
            <li key={m.name}>
              {m.name}: wanted {m.wanted}, board has {m.board ?? "no answer"}
            </li>
          ))}
        </ul>
      )}
    </>
  );
}
