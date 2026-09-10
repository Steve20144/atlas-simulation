import { Line } from "@react-three/drei";
import { useMemo } from "react";
import { Quaternion, Vector3 } from "three";
import { effectiveFan, frdToScene } from "../geometry";
import type { Fan, Foil, Vec3 } from "../types";

/** Duct radius in metres (80 mm rotor, 84 mm duct, PLAN.md section 3). */
const DUCT_RADIUS_M = 0.042;
const DUCT_LENGTH_M = 0.13;
/** Thrust vector length at u = 1, metres. */
const THRUST_SCALE_M = 0.35;

const Y_UP = new Vector3(0, 1, 0);

interface Props {
  fans: Fan[];
  foils?: Foil[];
  /** Normalised commands 0..1 per fan (hover u); undefined draws a nominal 0.5 vector. */
  u?: number[];
}

function colourForU(u: number): string {
  if (u >= 0.95) return "#f43f5e";
  if (u >= 0.75) return "#f59e0b";
  return "#34d399";
}

/**
 * Motor ducts at the motor positions along the motor axis; for foil motors a dotted jet path to the
 * foil pressure point; thrust arrows (force on the airframe) from where the force acts.
 */
export default function FanMarkers({ fans, foils, u }: Props) {
  const items = useMemo(
    () =>
      fans.map((fan, i) => {
        const eff = effectiveFan(fan, foils);
        const motorDir = new Vector3(...frdToScene(eff.motorAxis)).normalize();
        const dir = new Vector3(...frdToScene(eff.axis)).normalize();
        const motorPos = new Vector3(...frdToScene(fan.pos_frd_m));
        const forcePos = new Vector3(...frdToScene(eff.pos));
        const cmd = u?.[i] ?? 0.5;
        const tip = forcePos.clone().addScaledVector(dir, Math.max(cmd, 0.02) * THRUST_SCALE_M * eff.ctScale);
        const quat = new Quaternion().setFromUnitVectors(Y_UP, motorDir);
        return { fan, motorPos, forcePos, tip, quat, cmd, colour: colourForU(cmd), foil: eff.deflectionDeg !== null };
      }),
    [fans, foils, u],
  );

  return (
    <group>
      {items.map(({ fan, motorPos, forcePos, tip, quat, colour, foil }) => (
        <group key={fan.id}>
          <mesh position={motorPos} quaternion={quat}>
            <cylinderGeometry args={[DUCT_RADIUS_M, DUCT_RADIUS_M, DUCT_LENGTH_M, 24, 1, true]} />
            <meshStandardMaterial color="#94a3b8" side={2} metalness={0.2} roughness={0.6} />
          </mesh>
          {foil && (
            <>
              <Line points={[motorPos.toArray() as Vec3, forcePos.toArray() as Vec3]} color="#64748b" lineWidth={1} dashed dashSize={0.01} gapSize={0.01} />
              <mesh position={forcePos}>
                <boxGeometry args={[0.02, 0.06, 0.09]} />
                <meshStandardMaterial color="#475569" transparent opacity={0.7} />
              </mesh>
            </>
          )}
          <Line points={[forcePos.toArray() as Vec3, tip.toArray() as Vec3]} color={colour} lineWidth={2} />
          <mesh position={tip}>
            <sphereGeometry args={[0.012, 12, 12]} />
            <meshBasicMaterial color={colour} />
          </mesh>
        </group>
      ))}
    </group>
  );
}
