import { Grid, Line, OrbitControls } from "@react-three/drei";
import { Canvas } from "@react-three/fiber";
import { useState } from "react";
import { frdToScene } from "../geometry";
import { useTiltlabStore } from "../store";
import type { Vec3 } from "../types";
import Airflow from "./Airflow";
import CadModel, { CadErrorBoundary } from "./CadModel";
import FanMarkers from "./FanMarkers";
import ViewerOverlay from "./ViewerOverlay";

const AXIS_LEN_M = 0.3;
const ORIGIN: Vec3 = [0, 0, 0];

/** FRD axes triad: X forward red, Y right green, Z down blue, drawn in scene coordinates. */
function AxesTriad() {
  const axes: { frd: Vec3; colour: string }[] = [
    { frd: [AXIS_LEN_M, 0, 0], colour: "#ef4444" },
    { frd: [0, AXIS_LEN_M, 0], colour: "#22c55e" },
    { frd: [0, 0, AXIS_LEN_M], colour: "#3b82f6" },
  ];
  return (
    <group>
      {axes.map((a) => (
        <Line key={a.colour} points={[ORIGIN, frdToScene(a.frd)]} color={a.colour} lineWidth={2} />
      ))}
    </group>
  );
}

/** Centre panel: stick model of the aircraft with thrust vectors scaled by hover u. */
export default function Viewer3D() {
  const fans = useTiltlabStore((s) => s.scenario.fans);
  const foils = useTiltlabStore((s) => s.scenario.foils);
  const scenarioName = useTiltlabStore((s) => s.scenario.meta.name);
  const cadModel = useTiltlabStore((s) => s.scenario.meta.cad_model);
  const showCad = useTiltlabStore((s) => s.view.cad);
  const showFlow = useTiltlabStore((s) => s.view.flow);
  const [cadError, setCadError] = useState<string | null>(null);
  const thrustN = useTiltlabStore((s) => s.metrics?.hover?.thrust_N);
  const fanCurves = useTiltlabStore((s) => s.scenario.fan_curves);
  const maxThrustN = Math.max(
    1,
    ...Object.values(fanCurves).map((c) => c.points[c.points.length - 1]?.thrust_N ?? 0),
  );
  const cg = useTiltlabStore((s) => s.scenario.mass.cg_frd_m);
  const u = useTiltlabStore((s) => s.metrics?.hover?.u);

  return (
    <section className="relative h-full min-h-[320px] overflow-hidden rounded-xl" style={{ background: "var(--ui-panel)", border: "1px solid var(--ui-line-soft)" }}>
      <Canvas camera={{ position: [1.2, 0.9, 1.4], fov: 45, near: 0.01, far: 50 }}>
        <ambientLight intensity={0.6} />
        <directionalLight position={[2, 3, 2]} intensity={1.2} />
        <Grid
          args={[3, 3]}
          cellSize={0.1}
          sectionSize={0.5}
          cellColor="#1f1f1f"
          sectionColor="#333333"
          fadeDistance={6}
          infiniteGrid
        />
        <AxesTriad />
        <mesh position={frdToScene(cg)}>
          <sphereGeometry args={[0.02, 16, 16]} />
          <meshStandardMaterial color="#fbbf24" emissive="#f59e0b" emissiveIntensity={0.4} />
        </mesh>
        <FanMarkers fans={fans} foils={foils} u={u} />
        {showFlow && fans.length > 0 && (
          <Airflow fans={fans} foils={foils} thrustN={thrustN} maxThrustN={maxThrustN} />
        )}
        {cadModel && showCad && (
          <CadErrorBoundary key={scenarioName} onError={setCadError}>
            <CadModel url={`/api/cad/model/${encodeURIComponent(scenarioName)}`} onError={setCadError} />
          </CadErrorBoundary>
        )}
        <OrbitControls makeDefault enableDamping dampingFactor={0.1} />
      </Canvas>
      <ViewerOverlay maxThrustN={maxThrustN} cadError={cadError} />
    </section>
  );
}
