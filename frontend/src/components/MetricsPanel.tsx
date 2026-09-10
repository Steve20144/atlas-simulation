import type { ReactNode } from "react";
import { useTiltlabStore } from "../store";
import type { MetricGroup } from "../types";
import AuthorityTable, { Badge } from "./AuthorityTable";
import CouplingTable from "./CouplingTable";
import ExportPanel from "./ExportPanel";
import { fmt } from "./format";
import ParamsPreview from "./ParamsPreview";

const GROUPS: { id: MetricGroup; label: string }[] = [
  { id: "hover", label: "Hover" },
  { id: "authority", label: "Authority" },
  { id: "coupling", label: "Coupling" },
  { id: "conditioning", label: "Conditioning" },
  { id: "composite", label: "Composite" },
];

function Group({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="rounded border border-slate-800 p-2">
      <h3 className="mb-1 text-xs font-semibold">{title}</h3>
      {children}
    </div>
  );
}

/** Right panel: metric groups with show/hide tick boxes, badges, estimated banner, params preview. */
export default function MetricsPanel() {
  const metrics = useTiltlabStore((s) => s.metrics);
  const visible = useTiltlabStore((s) => s.visibleGroups);
  const toggleGroup = useTiltlabStore((s) => s.toggleGroup);
  const massEstimated = useTiltlabStore((s) => s.scenario.mass.estimated ?? false);
  const estimated = Boolean(metrics?.estimated) || massEstimated;

  return (
    <section className="flex h-full flex-col gap-2 overflow-auto border-l border-slate-700 bg-slate-900/60 p-2">
      <h2 className="text-sm font-semibold">Metrics</h2>
      {estimated && (
        <div
          role="status"
          className="rounded border border-amber-600 bg-amber-900/40 px-2 py-1 text-xs text-amber-200"
        >
          Estimated inputs ({(metrics?.estimated_sources ?? ["mass"]).join(", ")}): placeholders,
          not measured. Treat numbers as relative, not absolute.
        </div>
      )}
      {metrics?.notes?.map((n) => (
        <p key={n} className="text-[10px] text-rose-300">
          {n}
        </p>
      ))}
      <div className="flex flex-wrap gap-2 text-xs">
        {GROUPS.map((g) => (
          <label key={g.id} className="flex items-center gap-1">
            <input type="checkbox" checked={visible[g.id]} onChange={() => toggleGroup(g.id)} />
            {g.label}
          </label>
        ))}
      </div>

      {!metrics && <p className="text-xs text-slate-500">No metrics yet.</p>}

      {metrics && visible.hover && (
        <Group title="Hover">
          <div className="grid grid-cols-2 gap-x-2 text-xs">
            <span className="text-slate-400">power</span>
            <span className="tabular-nums">{fmt(metrics.hover?.power_W, 0)} W</span>
            <span className="text-slate-400">headroom</span>
            <span className="tabular-nums">{fmt(metrics.hover?.headroom, 3)}</span>
            <span className="text-slate-400">max u</span>
            <span className="tabular-nums">{fmt(Math.max(...(metrics.hover?.u ?? [NaN])))}</span>
            <span className="text-slate-400">total thrust</span>
            <span className="tabular-nums">
              {fmt((metrics.hover?.thrust_N ?? []).reduce((a, b) => a + b, 0), 1)} N
            </span>
          </div>
        </Group>
      )}

      {metrics && visible.authority && metrics.authority && (
        <Group title="Authority">
          <AuthorityTable authority={metrics.authority} />
        </Group>
      )}

      {metrics && visible.coupling && metrics.coupling && (
        <Group title="Coupling">
          <CouplingTable coupling={metrics.coupling} />
        </Group>
      )}

      {metrics && visible.conditioning && metrics.conditioning && (
        <Group title="Conditioning">
          <div className="grid grid-cols-2 gap-x-2 text-xs">
            <span className="text-slate-400">rank</span>
            <span className="tabular-nums">{metrics.conditioning.rank}</span>
            <span className="text-slate-400">nullity</span>
            <span className="tabular-nums">{metrics.conditioning.null_space_dim}</span>
            <span className="text-slate-400">condition number</span>
            <span className="tabular-nums">{fmt(metrics.conditioning.condition_number, 1)}</span>
            <span className="text-slate-400">singular values</span>
            <span className="tabular-nums">
              {(metrics.conditioning.singular_values ?? []).map((s) => fmt(s, 3)).join(", ")}
            </span>
          </div>
        </Group>
      )}

      {metrics && visible.composite && metrics.score && (
        <Group title="Composite">
          <div className="flex items-center gap-2 text-xs">
            <span className="text-2xl font-semibold tabular-nums">{fmt(metrics.score.value, 1)}</span>
            <Badge label={metrics.score.value >= 0.7 ? "good" : metrics.score.value >= 0.4 ? "weak" : "none"} />
          </div>
          <p className="mt-1 text-[10px] text-slate-500">
            weights{" "}
            {Object.entries(metrics.score.weights ?? {})
              .map(([k, v]) => `${k} ${fmt(v)}`)
              .join(", ")}
          </p>
        </Group>
      )}

      <ParamsPreview />
      <ExportPanel />
    </section>
  );
}
