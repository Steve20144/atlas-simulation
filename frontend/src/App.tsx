import GeometryPanel from "./components/GeometryPanel";
import MetricsPanel from "./components/MetricsPanel";
import TopBar from "./components/TopBar";
import Viewer3D from "./components/Viewer3D";

export const APP_NAME = "tiltlab";

export default function App() {
  return (
    <div className="flex h-screen flex-col">
      <TopBar />
      <main className="grid min-h-0 flex-1 grid-cols-[380px_minmax(0,1fr)_400px]">
        <GeometryPanel />
        <Viewer3D />
        <MetricsPanel />
      </main>
    </div>
  );
}
