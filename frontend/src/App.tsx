import GeometryPanel from "./components/GeometryPanel";
import MetricsPanel from "./components/MetricsPanel";
import SideRail from "./components/SideRail";
import TopBar from "./components/TopBar";
import Viewer3D from "./components/Viewer3D";
import { useTiltlabStore } from "./store";

export const APP_NAME = "tiltlab";

export default function App() {
  const showGeometry = useTiltlabStore((s) => s.view.geometry);
  const showMetrics = useTiltlabStore((s) => s.view.metrics);
  const columns = [showGeometry ? "minmax(280px, 24%)" : null, "minmax(0,1fr)", showMetrics ? "minmax(320px, 27%)" : null]
    .filter(Boolean)
    .join(" ");

  return (
    <div className="flex h-screen flex-col" style={{ background: "var(--ui-bg)" }}>
      <TopBar />
      <div className="flex min-h-0 flex-1">
        <SideRail />
        <main className="grid min-h-0 flex-1 gap-2 p-2 pl-0" style={{ gridTemplateColumns: columns }}>
          {showGeometry && <GeometryPanel />}
          <Viewer3D />
          {showMetrics && <MetricsPanel />}
        </main>
      </div>
    </div>
  );
}
