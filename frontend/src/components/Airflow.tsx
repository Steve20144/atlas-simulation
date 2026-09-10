import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import { BufferAttribute, BufferGeometry, CatmullRomCurve3, Vector3 } from "three";
import { turbo } from "../colormap";
import { effectiveFan, frdToScene } from "../geometry";
import type { Fan, Foil } from "../types";

const PARTICLES_PER_FAN = 56;
const DUCT_HALF_LENGTH_M = 0.065;
const EXHAUST_LENGTH_M = 0.4;
const AIR_DENSITY = 1.225;
const DUCT_AREA_M2 = Math.PI * 0.04 * 0.04;

interface Props {
  fans: Fan[];
  foils?: Foil[];
  /** Motor thrust per fan in N (hover); drives colour and particle speed. */
  thrustN?: number[];
  /** Full-scale thrust for the colour map, N. */
  maxThrustN: number;
  speed?: number;
}

/** Jet velocity from momentum theory, m/s: T = rho * A * v^2. */
export function jetVelocity(thrustN: number): number {
  return Math.sqrt(Math.max(0, thrustN) / (AIR_DENSITY * DUCT_AREA_M2));
}

/**
 * Path of the air through one fan: intake, duct exit, then (for a foil motor) into the foil
 * channel to the pressure point and out along the deflected exhaust direction. Scene coordinates.
 */
function flowPath(fan: Fan, foils: Foil[] | undefined): CatmullRomCurve3 {
  const eff = effectiveFan(fan, foils);
  const motor = new Vector3(...frdToScene(fan.pos_frd_m));
  const motorAxis = new Vector3(...frdToScene(eff.motorAxis)).normalize(); // thrust direction
  const exhaust = new Vector3(...frdToScene(eff.axis)).normalize().negate(); // where the air goes
  const intake = motor.clone().addScaledVector(motorAxis, DUCT_HALF_LENGTH_M);
  const exit = motor.clone().addScaledVector(motorAxis, -DUCT_HALF_LENGTH_M);
  const points = [intake, motor, exit];
  if (eff.deflectionDeg !== null) {
    const pressure = new Vector3(...frdToScene(eff.pos));
    points.push(exit.clone().lerp(pressure, 0.5), pressure);
    points.push(pressure.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M * 0.35));
    points.push(pressure.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M));
  } else {
    points.push(exit.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M * 0.5));
    points.push(exit.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M));
  }
  return new CatmullRomCurve3(points, false, "catmullrom", 0.2);
}

/** Animated particles that follow the air through each duct and foil, coloured by jet intensity. */
export default function Airflow({ fans, foils, thrustN, maxThrustN, speed = 1 }: Props) {
  const paths = useMemo(() => fans.map((f) => flowPath(f, foils)), [fans, foils]);
  const count = fans.length * PARTICLES_PER_FAN;
  const geometry = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute("position", new BufferAttribute(new Float32Array(count * 3), 3));
    g.setAttribute("color", new BufferAttribute(new Float32Array(count * 3), 3));
    return g;
  }, [count]);
  const phase = useRef(0);
  const vMax = jetVelocity(maxThrustN) || 1;
  const tmp = new Vector3();

  useFrame((_, dt) => {
    phase.current += dt * speed;
    const pos = geometry.getAttribute("position") as BufferAttribute;
    const col = geometry.getAttribute("color") as BufferAttribute;
    fans.forEach((_, fi) => {
      const T = thrustN?.[fi] ?? maxThrustN * 0.5;
      const v = jetVelocity(T);
      const cyclesPerSecond = 0.25 + 1.25 * (v / vMax);
      const [r, g, b] = turbo(maxThrustN > 0 ? T / maxThrustN : 0.5);
      for (let k = 0; k < PARTICLES_PER_FAN; k++) {
        const i = fi * PARTICLES_PER_FAN + k;
        const t = (k / PARTICLES_PER_FAN + phase.current * cyclesPerSecond) % 1;
        paths[fi].getPointAt(t, tmp);
        pos.setXYZ(i, tmp.x, tmp.y, tmp.z);
        const fade = t > 0.8 ? 1 - (t - 0.8) / 0.2 : 1; // dim towards the end of the exhaust
        col.setXYZ(i, r * fade, g * fade, b * fade);
      }
    });
    pos.needsUpdate = true;
    col.needsUpdate = true;
  });

  return (
    <points geometry={geometry} frustumCulled={false}>
      <pointsMaterial vertexColors size={0.014} sizeAttenuation transparent opacity={0.95} depthWrite={false} />
    </points>
  );
}
