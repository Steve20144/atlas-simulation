import { PRESETS } from "../presets";
import { useTiltlabStore } from "../store";
import { turboGradient } from "../colormap";
import { jetVelocity } from "./Airflow";
import { fmt } from "./format";
import HoverPitchControl from "./HoverPitchControl";

/** Unit card, as the drone card in the reference: identity plus the numbers that matter at a glance. */
function UnitCard() {
  const name = useTiltlabStore((s) => s.scenario.meta.name);
  const concept = useTiltlabStore((s) => s.concept);
  const massKg = useTiltlabStore((s) => s.scenario.mass.total_kg);
  const estimated = useTiltlabStore((s) => s.scenario.mass.estimated ?? false);
  const hover = useTiltlabStore((s) => s.metrics?.hover);
  const fans = useTiltlabStore((s) => s.scenario.fans.length);
  const board = useTiltlabStore((s) => s.board.status);
  const maxU = hover ? Math.max(...hover.u) : NaN;
  const headroom = hover?.headroom ?? NaN;
  const dot = !hover ? "" : headroom > 0.25 ? "ui-dot-ok" : headroom > 0.1 ? "ui-dot-warn" : "ui-dot-bad";

  return (
    <div className="ui-float pointer-events-auto w-[230px] p-3">
      <div className="flex items-center gap-2">
        <span className={`ui-dot ${dot}`} />
        <span className="ui-label">airframe</span>
        <span className="ml-auto truncate font-mono text-[11px]">{name}</span>
      </div>
      <dl className="mt-3 grid grid-cols-[auto_1fr] gap-x-3 gap-y-1 text-[11px]">
        <dt className="ui-label self-center">concept</dt>
        <dd className="text-right font-mono">{concept === "stock" ? "stock px4" : "fully actuated"}</dd>
        <dt className="ui-label self-center">fans</dt>
        <dd className="text-right tabular-nums">{fans}</dd>
        <dt className="ui-label self-center">mass</dt>
        <dd className="text-right tabular-nums">
          {massKg.toFixed(2)} kg{estimated ? " est." : ""}
        </dd>
        <dt className="ui-label self-center">hover power</dt>
        <dd className="text-right tabular-nums">{hover ? `${fmt(hover.power_W, 0)} W` : "..."}</dd>
        <dt className="ui-label self-center">headroom</dt>
        <dd className="text-right tabular-nums">{hover ? fmt(headroom, 3) : "..."}</dd>
        <dt className="ui-label self-center">max u</dt>
        <dd className="text-right tabular-nums">{hover ? fmt(maxU) : "..."}</dd>
        <dt className="ui-label self-center">board</dt>
        <dd className="text-right font-mono" style={{ color: board?.connected ? "var(--ui-ok)" : "var(--ui-muted)" }}>
          {board === null ? "not checked" : board.connected ? board.port : "offline"}
        </dd>
      </dl>
      {hover && (
        <div className="mt-3">
          <div className="flex justify-between">
            <span className="ui-label">fan load</span>
            <span className="ui-label">{fmt(maxU * 100, 0)}%</span>
          </div>
          <div className="mt-1 h-1 w-full overflow-hidden rounded-full" style={{ background: "var(--ui-line)" }}>
            <div className="h-full rounded-full" style={{ width: `${Math.min(100, maxU * 100)}%`, background: "var(--ui-text)" }} />
          </div>
        </div>
      )}
    </div>
  );
}

/** Collective and hover pitch, the two trim inputs, as a pill row (City / District / Street). */
function TrimRow() {
  const collective = useTiltlabStore((s) => s.collective);
  const collectiveHover = useTiltlabStore((s) => s.metrics?.collective_hover);
  const setCollective = useTiltlabStore((s) => s.setCollective);
  const sliderValue = collective ?? collectiveHover ?? 0.5;
  return (
    <div className="ui-float pointer-events-auto flex flex-wrap items-center gap-3 px-3 py-1.5">
      <label className="flex items-center gap-2">
        <span className="ui-label">Collective</span>
        <input aria-label="Collective" type="range" min={0} max={1} step={0.01} value={sliderValue} onChange={(e) => setCollective(Number(e.target.value))} />
        <span className="w-9 tabular-nums text-[11px]">{sliderValue.toFixed(2)}</span>
        <button className={collective === null ? "ui-btn-active" : "ui-btn"} onClick={() => setCollective(null)} title="Let the backend solve for hover">
          hover
        </button>
      </label>
      <span className="h-4 w-px" style={{ background: "var(--ui-line)" }} />
      <HoverPitchControl />
    </div>
  );
}

/** Bottom strip: presets as unit pills plus the legend for the jet colour scale. */
function BottomStrip({ maxThrustN }: { maxThrustN: number }) {
  const applyPreset = useTiltlabStore((s) => s.applyPreset);
  const showFlow = useTiltlabStore((s) => s.view.flow);
  return (
    <div className="ui-float pointer-events-auto flex flex-wrap items-center gap-2 px-3 py-2">
      <span className="ui-label mr-1">presets</span>
      {PRESETS.map((p) => (
        <button key={p.id} className="ui-btn" title={p.title} onClick={() => void applyPreset(p.id)}>
          <span className="ui-dot ui-dot-ok" />
          {p.label}
        </button>
      ))}
      <span className="ml-auto ui-label">
        <span style={{ color: "#ef4444" }}>X</span> fwd <span style={{ color: "#22c55e" }}>Y</span> right{" "}
        <span style={{ color: "#3b82f6" }}>Z</span> down (FRD)
      </span>
      {showFlow && (
        <span className="flex items-center gap-2">
          <span className="ui-label">jet 0</span>
          <span className="h-1.5 w-24 rounded-full" style={{ background: turboGradient() }} />
          <span className="ui-label tabular-nums">
            {maxThrustN.toFixed(0)} N, {jetVelocity(maxThrustN).toFixed(0)} m/s
          </span>
        </span>
      )}
    </div>
  );
}

/** Everything drawn over the 3D canvas; pointer events pass through except on the cards. */
export default function ViewerOverlay({ maxThrustN, cadError }: { maxThrustN: number; cadError: string | null }) {
  return (
    <div className="pointer-events-none absolute inset-0 flex flex-col p-3">
      <div className="flex items-start gap-3">
        <UnitCard />
        <div className="flex flex-1 justify-center">
          <TrimRow />
        </div>
      </div>
      <div className="mt-auto">
        {cadError && (
          <p className="ui-float mb-2 px-3 py-1 text-[10px]" style={{ color: "var(--ui-bad)" }}>
            CAD model failed to load: {cadError}
          </p>
        )}
        <BottomStrip maxThrustN={maxThrustN} />
      </div>
    </div>
  );
}
