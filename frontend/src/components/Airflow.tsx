import { useFrame } from "@react-three/fiber";
import { useMemo, useRef } from "react";
import {
  AdditiveBlending,
  BufferAttribute,
  BufferGeometry,
  CanvasTexture,
  CatmullRomCurve3,
  Color,
  DoubleSide,
  Quaternion,
  Vector3,
} from "three";
import { turbo } from "../colormap";
import { effectiveFan, frdToScene } from "../geometry";
import type { Fan, Foil } from "../types";

const PARTICLES_PER_FAN = 140;
const DUCT_HALF_LENGTH_M = 0.065;
const DUCT_RADIUS_M = 0.04;
const EXHAUST_LENGTH_M = 0.7;
const AIR_DENSITY = 1.225;
const DUCT_AREA_M2 = Math.PI * DUCT_RADIUS_M * DUCT_RADIUS_M;
const Y_UP = new Vector3(0, 1, 0);

interface Props {
  fans: Fan[];
  foils?: Foil[];
  /** Motor thrust per fan in N (hover); drives colour, speed, particle size and plume. */
  thrustN?: number[];
  /** Full-scale thrust for the colour map, N. */
  maxThrustN: number;
  speed?: number;
}

/** Jet velocity from momentum theory, m/s: T = rho * A * v^2. */
export function jetVelocity(thrustN: number): number {
  return Math.sqrt(Math.max(0, thrustN) / (AIR_DENSITY * DUCT_AREA_M2));
}

interface FlowPath {
  curve: CatmullRomCurve3;
  /** Fraction of the curve inside the duct (particles stay within the duct radius there). */
  ductEnd: number;
  /** Where the exhaust plume starts (duct exit or foil pressure point) and its direction. */
  plumeStart: Vector3;
  exhaust: Vector3;
}

/**
 * Path of the air through one fan in scene coordinates: intake, through the duct, then for a foil
 * motor into the channel to the pressure point and out along the deflected exhaust direction.
 */
function flowPath(fan: Fan, foils: Foil[] | undefined): FlowPath {
  const eff = effectiveFan(fan, foils);
  const motor = new Vector3(...frdToScene(fan.pos_frd_m));
  const motorAxis = new Vector3(...frdToScene(eff.motorAxis)).normalize(); // thrust direction
  const exhaust = new Vector3(...frdToScene(eff.axis)).normalize().negate(); // where the air goes
  const intake = motor.clone().addScaledVector(motorAxis, DUCT_HALF_LENGTH_M * 1.6);
  const exit = motor.clone().addScaledVector(motorAxis, -DUCT_HALF_LENGTH_M);
  const points = [intake, motor, exit];
  let plumeStart = exit;
  if (eff.deflectionDeg !== null) {
    const pressure = new Vector3(...frdToScene(eff.pos));
    points.push(exit.clone().lerp(pressure, 0.5), pressure);
    points.push(pressure.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M * 0.3));
    points.push(pressure.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M));
    plumeStart = pressure;
  } else {
    points.push(exit.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M * 0.4));
    points.push(exit.clone().addScaledVector(exhaust, EXHAUST_LENGTH_M));
  }
  const curve = new CatmullRomCurve3(points, false, "catmullrom", 0.25);
  // arc-length fraction of the duct exit along the whole path
  const lengths = curve.getLengths(200);
  const total = lengths[lengths.length - 1];
  const ductLength = intake.distanceTo(motor) + motor.distanceTo(exit);
  return { curve, ductEnd: Math.min(0.95, ductLength / total), plumeStart, exhaust };
}

/** Soft round sprite so particles glow instead of drawing as hard squares. */
function makeSprite(): CanvasTexture {
  const size = 64;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d");
  if (ctx) {
    const g = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    g.addColorStop(0, "rgba(255,255,255,1)");
    g.addColorStop(0.35, "rgba(255,255,255,0.6)");
    g.addColorStop(1, "rgba(255,255,255,0)");
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, size, size);
  }
  const tex = new CanvasTexture(canvas);
  tex.needsUpdate = true;
  return tex;
}

/** Deterministic per-particle jitter (angle and radius fraction) so the jet has a body. */
function seedJitter(count: number): Float32Array {
  const out = new Float32Array(count * 2);
  let s = 12345;
  const rnd = () => {
    s = (s * 1103515245 + 12345) & 0x7fffffff;
    return s / 0x7fffffff;
  };
  for (let i = 0; i < count; i++) {
    out[2 * i] = rnd() * Math.PI * 2;
    out[2 * i + 1] = Math.sqrt(rnd());
  }
  return out;
}

/**
 * CFD-style animated jets: glowing particles with speed trails that follow the air through each
 * duct and foil, spread to the duct radius and widening downstream, plus a translucent plume cone.
 * Colour, speed, size and plume opacity all follow the motor's hover thrust on the turbo scale.
 */
export default function Airflow({ fans, foils, thrustN, maxThrustN, speed = 1 }: Props) {
  const paths = useMemo(() => fans.map((f) => flowPath(f, foils)), [fans, foils]);
  const count = fans.length * PARTICLES_PER_FAN;
  const sprite = useMemo(makeSprite, []);
  const jitter = useMemo(() => seedJitter(count), [count]);
  const pointsGeom = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute("position", new BufferAttribute(new Float32Array(count * 3), 3));
    g.setAttribute("color", new BufferAttribute(new Float32Array(count * 3), 3));
    return g;
  }, [count]);
  const trailGeom = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute("position", new BufferAttribute(new Float32Array(count * 6), 3));
    g.setAttribute("color", new BufferAttribute(new Float32Array(count * 6), 3));
    return g;
  }, [count]);
  const phase = useRef(0);
  const vMax = jetVelocity(maxThrustN) || 1;
  const tmp = useMemo(() => ({ p: new Vector3(), tan: new Vector3(), n1: new Vector3(), n2: new Vector3(), q: new Vector3() }), []);

  const plumes = useMemo(
    () =>
      paths.map((path, fi) => {
        const T = thrustN?.[fi] ?? 0;
        const intensity = maxThrustN > 0 ? T / maxThrustN : 0;
        const len = EXHAUST_LENGTH_M * (0.35 + 0.65 * intensity);
        const mid = path.plumeStart.clone().addScaledVector(path.exhaust, len / 2);
        const quat = new Quaternion().setFromUnitVectors(Y_UP, path.exhaust.clone().negate());
        const [r, g, b] = turbo(intensity);
        return { mid, quat, len, colour: new Color(r, g, b), intensity };
      }),
    [paths, thrustN, maxThrustN],
  );

  useFrame((_, dt) => {
    phase.current += dt * speed;
    const pos = pointsGeom.getAttribute("position") as BufferAttribute;
    const col = pointsGeom.getAttribute("color") as BufferAttribute;
    const tpos = trailGeom.getAttribute("position") as BufferAttribute;
    const tcol = trailGeom.getAttribute("color") as BufferAttribute;
    const { p, tan, n1, n2, q } = tmp;
    paths.forEach((path, fi) => {
      const T = thrustN?.[fi] ?? 0;
      const intensity = maxThrustN > 0 ? T / maxThrustN : 0;
      const v = jetVelocity(T);
      const cyclesPerSecond = 0.15 + 2.2 * (v / vMax);
      const [r, g, b] = turbo(intensity);
      const trailLen = 0.015 + 0.06 * (v / vMax);
      for (let k = 0; k < PARTICLES_PER_FAN; k++) {
        const i = fi * PARTICLES_PER_FAN + k;
        const t = (k / PARTICLES_PER_FAN + phase.current * cyclesPerSecond) % 1;
        path.curve.getPointAt(t, p);
        path.curve.getTangentAt(t, tan);
        // perpendicular basis for the jitter disc
        n1.crossVectors(tan, Math.abs(tan.y) < 0.9 ? Y_UP : new Vector3(1, 0, 0)).normalize();
        n2.crossVectors(tan, n1).normalize();
        const spread = t < path.ductEnd ? DUCT_RADIUS_M * 0.85 : DUCT_RADIUS_M * (0.85 + 1.6 * (t - path.ductEnd));
        const a = jitter[2 * i];
        const rad = jitter[2 * i + 1] * spread;
        q.copy(p).addScaledVector(n1, Math.cos(a) * rad).addScaledVector(n2, Math.sin(a) * rad);
        pos.setXYZ(i, q.x, q.y, q.z);
        const fade = (t > 0.75 ? 1 - (t - 0.75) / 0.25 : 1) * (0.35 + 0.65 * intensity);
        col.setXYZ(i, r * fade, g * fade, b * fade);
        // trail: from the particle back along the tangent, length grows with jet speed
        tpos.setXYZ(2 * i, q.x, q.y, q.z);
        tpos.setXYZ(2 * i + 1, q.x - tan.x * trailLen, q.y - tan.y * trailLen, q.z - tan.z * trailLen);
        tcol.setXYZ(2 * i, r * fade, g * fade, b * fade);
        tcol.setXYZ(2 * i + 1, 0, 0, 0);
      }
    });
    pos.needsUpdate = true;
    col.needsUpdate = true;
    tpos.needsUpdate = true;
    tcol.needsUpdate = true;
  });

  return (
    <group>
      <points geometry={pointsGeom} frustumCulled={false}>
        <pointsMaterial
          vertexColors
          map={sprite}
          size={0.045}
          sizeAttenuation
          transparent
          opacity={0.95}
          depthWrite={false}
          blending={AdditiveBlending}
        />
      </points>
      <lineSegments geometry={trailGeom} frustumCulled={false}>
        <lineBasicMaterial vertexColors transparent opacity={0.8} depthWrite={false} blending={AdditiveBlending} />
      </lineSegments>
      {plumes.map((pl, i) =>
        pl.intensity > 0.01 ? (
          <mesh key={i} position={pl.mid} quaternion={pl.quat}>
            <coneGeometry args={[DUCT_RADIUS_M * 2.4, pl.len, 24, 1, true]} />
            <meshBasicMaterial
              color={pl.colour}
              transparent
              opacity={0.08 + 0.14 * pl.intensity}
              side={DoubleSide}
              depthWrite={false}
              blending={AdditiveBlending}
            />
          </mesh>
        ) : null,
      )}
    </group>
  );
}
