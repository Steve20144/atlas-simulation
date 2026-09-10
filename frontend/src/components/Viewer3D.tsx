import { Grid, Line, OrbitControls } from "@react-three/drei";
import { Canvas } from "@react-three/fiber";
import { useState } from "react";
import { frdToScene } from "../geometry";
import { useTiltlabStore } from "../store";
import type { Vec3 } from "../types";
import { turboGradient } from "../colormap";
import Airflow, { jetVelocity } from "./Airflow";
import CadModel, { CadErrorBoundary } from "./CadModel";
import FanMarkers from "./FanMarkers";

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
  const [showCad, setShowCad] = useState(true);
  const [showFlow, setShowFlow] = useState(true);
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
    <section className="relative h-full min-h-[320px] bg-slate-950">
      <Canvas camera={{ position: [1.2, 0.9, 1.4], fov: 45, near: 0.01, far: 50 }}>
        <ambientLight intensity={0.6} />
        <directionalLight position={[2, 3, 2]} intensity={1.2} />
        <Grid
          args={[3, 3]}
          cellSize={0.1}
          sectionSize={0.5}
          cellColor="#334155"
          sectionColor="#475569"
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
      <div className="pointer-events-none absolute left-2 top-2 rounded bg-slate-900/70 px-2 py-1 text-[10px] leading-snug text-slate-300">
        <div>
          <span className="text-red-400">X</span> fwd <span className="text-green-400">Y</span> right{" "}
          <span className="text-blue-400">Z</span> down (FRD)
        </div>
        <div>
          <span className="text-amber-300">CG</span> marker. Grey ducts are the motors; the dotted line is the
          jet into the foil; arrows start where the force acts and point where it pushes the aircraft,
          scaled by hover u.
        </div>
        {cadModel && (
          <label className="pointer-events-auto mt-1 flex items-center gap-1">
            <input type="checkbox" checked={showCad} onChange={(e) => setShowCad(e.target.checked)} />
            show CAD airframe
          </label>
        )}
        {cadError && <div className="text-rose-300">CAD model failed to load: {cadError}</div>}
        <label className="pointer-events-auto mt-1 flex items-center gap-1">
          <input type="checkbox" checked={showFlow} onChange={(e) => setShowFlow(e.target.checked)} />
          animate airflow (particles follow the jet through the duct and foil)
        </label>
        {showFlow && (
          <div className="mt-1">
            <div className="flex justify-between">
              <span>0 N, 0 m/s</span>
              <span>jet thrust and velocity (momentum theory)</span>
              <span>
                {maxThrustN.toFixed(0)} N, {jetVelocity(maxThrustN).toFixed(0)} m/s
              </span>
            </div>
            <div className="h-2 w-full rounded" style={{ background: turboGradient() }} />
          </div>
        )}
      </div>
    </section>
  );
}
