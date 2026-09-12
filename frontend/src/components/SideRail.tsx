import { useTiltlabStore, type ViewFlag } from "../store";

/** Small line icons drawn inline so the rail needs no icon dependency. */
const ICONS: Record<string, JSX.Element> = {
  geometry: (
    <g>
      <rect x="3" y="3" width="7" height="7" rx="1.5" />
      <rect x="14" y="3" width="7" height="7" rx="1.5" />
      <rect x="3" y="14" width="7" height="7" rx="1.5" />
      <rect x="14" y="14" width="7" height="7" rx="1.5" />
    </g>
  ),
  metrics: (
    <g>
      <path d="M4 20h16" />
      <path d="M6 16l4-5 4 3 4-7" />
    </g>
  ),
  cad: (
    <g>
      <path d="M12 3l8 4.5v9L12 21l-8-4.5v-9L12 3z" />
      <path d="M12 12l8-4.5M12 12v9M12 12L4 7.5" />
    </g>
  ),
  flow: (
    <g>
      <path d="M3 8c3-2 6 2 9 0s6-2 9 0" />
      <path d="M3 14c3-2 6 2 9 0s6-2 9 0" />
    </g>
  ),
  board: (
    <g>
      <rect x="6" y="6" width="12" height="12" rx="2" />
      <path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4" />
    </g>
  ),
};

const TOGGLES: { flag: ViewFlag; title: string }[] = [
  { flag: "geometry", title: "Geometry panel" },
  { flag: "metrics", title: "Metrics and tools panel" },
  { flag: "cad", title: "CAD airframe in the 3D view" },
  { flag: "flow", title: "Animated airflow in the 3D view" },
];

function Icon({ name }: { name: string }) {
  return (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round">
      {ICONS[name]}
    </svg>
  );
}

/** Left icon rail: panel and layer toggles at the top, board connection at the bottom. */
export default function SideRail() {
  const view = useTiltlabStore((s) => s.view);
  const toggleView = useTiltlabStore((s) => s.toggleView);
  const connected = useTiltlabStore((s) => s.board.status?.connected ?? null);
  const checking = useTiltlabStore((s) => s.board.checking);
  const checkBoard = useTiltlabStore((s) => s.checkBoard);
  const dot = checking ? "ui-dot-busy" : connected === null ? "" : connected ? "ui-dot-ok" : "ui-dot-bad";

  return (
    <nav aria-label="View" className="flex w-12 flex-col items-center gap-1 py-2">
      {TOGGLES.map((t) => (
        <button
          key={t.flag}
          className="ui-rail-btn"
          aria-pressed={view[t.flag]}
          aria-label={t.title}
          title={t.title}
          onClick={() => toggleView(t.flag)}
        >
          <Icon name={t.flag} />
        </button>
      ))}
      <div className="mt-auto flex flex-col items-center gap-1">
        <button
          className="ui-rail-btn relative"
          aria-label="Check board connection"
          title="Check whether the Pixhawk is connected over USB"
          onClick={() => void checkBoard()}
        >
          <Icon name="board" />
          <span className={`ui-dot absolute right-1 top-1 ${dot}`} />
        </button>
      </div>
    </nav>
  );
}
