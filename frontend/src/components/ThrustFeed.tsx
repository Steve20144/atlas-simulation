import { useVectraStore } from "../store";
import type { ThrustSample } from "../types";

const pct = (v: number | null | undefined) => (v === null || v === undefined ? "--" : `${(v * 100).toFixed(0)}%`);

/** Two traces over the last 30 s: stick (cyan) and the thrust the controller asks for (orange). */
function Sparkline({ samples }: { samples: ThrustSample[] }) {
  if (samples.length < 2) return <div className="h-14 text-[10px] text-slate-500">waiting for samples</div>;
  const t0 = samples[0].t;
  const span = Math.max(1, samples[samples.length - 1].t - t0);
  const path = (pick: (s: ThrustSample) => number | null) =>
    samples
      .filter((s) => pick(s) !== null)
      .map((s) => `${(((s.t - t0) / span) * 100).toFixed(2)},${(56 - (pick(s) as number) * 52).toFixed(1)}`)
      .join(" ");
  return (
    <svg viewBox="0 0 100 60" preserveAspectRatio="none" className="h-14 w-full rounded" style={{ background: "var(--ui-panel-2, #0f172a)" }} role="img" aria-label="stick and thrust over time">
      <line x1="0" y1="56" x2="100" y2="56" stroke="#334155" strokeWidth="0.5" />
      <polyline points={path((s) => s.throttle)} fill="none" stroke="#22d3ee" strokeWidth="1" vectorEffect="non-scaling-stroke" />
      <polyline points={path((s) => s.thrust_sp)} fill="none" stroke="#f97316" strokeWidth="1" vectorEffect="non-scaling-stroke" />
    </svg>
  );
}

/** One ESC pulse width as a bar: 1000 us empty, 2000 us full. */
function OutputBar({ label, us }: { label: string; us: number }) {
  const w = Math.max(0, Math.min(100, ((us - 1000) / 1000) * 100));
  return (
    <div className="flex items-center gap-1 text-[10px] tabular-nums">
      <span className="w-12 text-slate-400">{label}</span>
      <div className="h-2 flex-1 rounded bg-slate-800">
        <div className="h-2 rounded" style={{ width: `${w}%`, background: us > 1000 ? "#f97316" : "#334155" }} />
      </div>
      <span className="w-10 text-right">{us}</span>
    </div>
  );
}

/**
 * Live view of the thrust path while the board runs: stick pulse width, PX4's throttle after
 * calibration and dead zone, the collective the attitude controller asks for, and the pulse
 * widths every ESC receives (MAIN = px4io outputs, AUX = fmu outputs). Polled from the
 * backend feed at 5 Hz. The board really drives its outputs meanwhile: fans and ESCs unpowered.
 */
export default function ThrustFeed() {
  const thrust = useVectraStore((s) => s.thrust);
  const stopThrustFeed = useVectraStore((s) => s.stopThrustFeed);
  const snap = thrust.snapshot;
  if (!thrust.running && !snap) return null;
  const lt = snap?.latest;
  const p = snap?.params ?? {};
  const fmt = (v: number | null | undefined, d = 2) => (v === null || v === undefined ? "--" : v.toFixed(d));
  return (
    <div className="flex flex-col gap-1 rounded border border-slate-700 p-2" data-testid="thrust-feed">
      <div className="flex items-center justify-between text-[11px]">
        <span className="flex items-center gap-2">
          <span className={`ui-dot ${lt?.armed ? "ui-dot-warn" : thrust.running ? "ui-dot-ok" : ""}`} />
          {lt?.armed ? "ARMED" : "disarmed"}
          {lt?.mode ? ` / ${lt.mode}` : ""}
          {lt?.hil ? " / HIL" : ""}
          <span className="text-slate-500">{snap ? `${snap.samples.length} samples, ${snap.uptime_s} s` : ""}</span>
        </span>
        <button className="ui-btn" onClick={() => void stopThrustFeed()}>
          Stop test
        </button>
      </div>
      <div className="grid grid-cols-3 gap-2 text-center tabular-nums">
        <div>
          <div className="ui-label">stick</div>
          <div className="text-lg">{lt?.rc_raw ?? "--"} us</div>
        </div>
        <div>
          <div className="ui-label">throttle {lt?.throttle_source === "MANUAL_CONTROL" ? "(PX4)" : "(RC)"}</div>
          <div className="text-lg" style={{ color: "#22d3ee" }}>{pct(lt?.throttle)}</div>
        </div>
        <div>
          <div className="ui-label">thrust setpoint</div>
          <div className="text-lg" style={{ color: "#f97316" }}>{pct(lt?.thrust_sp)}</div>
        </div>
      </div>
      <Sparkline samples={snap?.samples ?? []} />
      <div className="grid grid-cols-2 gap-x-3">
        <div>{(lt?.main_us ?? []).map((us, i) => <OutputBar key={`m${i}`} label={`MAIN ${i + 1}`} us={us} />)}</div>
        <div>{(lt?.aux_us ?? []).map((us, i) => <OutputBar key={`a${i}`} label={`AUX ${i + 1}`} us={us} />)}</div>
      </div>
      <p className="text-[10px] text-slate-500">
        hover {fmt(p.MPC_THR_HOVER)} | manual min {fmt(p.MPC_MANTHR_MIN)} | thr min {fmt(p.MPC_THR_MIN)} | max {fmt(p.MPC_THR_MAX)} | THR_MDL_FAC {fmt(p.THR_MDL_FAC)} | airmode {fmt(p.MC_AIRMODE, 0)} | spoolup {fmt(p.COM_SPOOLUP_TIME, 1)} s | RC3 {fmt(p.RC3_MIN, 0)}..{fmt(p.RC3_MAX, 0)} trim {fmt(p.RC3_TRIM, 0)} dz {fmt(p.RC3_DZ, 0)}
      </p>
      {(thrust.error || snap?.error) && (
        <p className="text-[10px]" style={{ color: "var(--ui-bad)" }}>
          {thrust.error ?? snap?.error}
        </p>
      )}
    </div>
  );
}
