import { Line } from "@react-three/drei";
import { useMemo } from "react";
import { Quaternion, Vector3 } from "three";
import { frdToScene, tiltAzimuthToAxis } from "../geometry";
import type { Fan, Vec3 } from "../types";

/** Duct radius in metres (80 mm rotor, 84 mm duct, PLAN.md section 3). */
const DUCT_RADIUS_M = 0.042;
const DUCT_LENGTH_M = 0.06;
/** Thrust vector length at u = 1, metres. */
const THRUST_SCALE_M = 0.35;

const Y_UP = new Vector3(0, 1, 0);

interface Props {
  fans: Fan[];
  /** Normalised commands 0..1 per fan (hover u); undefined draws a nominal 0.5 vector. */
  u?: number[];
}

function colourForU(u: number): string {
  if (u >= 0.95) return "#f43f5e";
  if (u >= 0.75) return "#f59e0b";
  return "#34d399";
}

/** Small cylinders at the fan positions, oriented along the thrust axis, plus thrust vectors. */
export default function FanMarkers({ fans, u }: Props) {
  const items = useMemo(
    () =>
      fans.map((fan, i) => {
        const axisFrd = tiltAzimuthToAxis(fan.tilt_deg, fan.azimuth_deg);
        const dir = new Vector3(...frdToScene(axisFrd)).normalize();
        const pos = new Vector3(...frdToScene(fan.pos_frd_m));
        const cmd = u?.[i] ?? 0.5;
        const tip = pos.clone().addScaledVector(dir, Math.max(cmd, 0.02) * THRUST_SCALE_M);
        const quat = new Quaternion().setFromUnitVectors(Y_UP, dir);
        return { fan, pos, tip, quat, cmd, colour: colourForU(cmd) };
      }),
    [fans, u],
  );

  return (
    <group>
      {items.map(({ fan, pos, tip, quat, colour }) => (
        <group key={fan.id}>
          <mesh position={pos} quaternion={quat}>
            <cylinderGeometry args={[DUCT_RADIUS_M, DUCT_RADIUS_M, DUCT_LENGTH_M, 24, 1, true]} />
            <meshStandardMaterial color="#94a3b8" side={2} metalness={0.2} roughness={0.6} />
          </mesh>
          <Line
            points={[pos.toArray() as Vec3, tip.toArray() as Vec3]}
            color={colour}
            lineWidth={2}
          />
          <mesh position={tip}>
            <sphereGeometry args={[0.012, 12, 12]} />
            <meshBasicMaterial color={colour} />
          </mesh>
        </group>
      ))}
    </group>
  );
}
