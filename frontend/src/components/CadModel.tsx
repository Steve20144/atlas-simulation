import { Component, type ReactNode, useEffect, useMemo, useState } from "react";
import { Box3, DoubleSide, Group, Matrix4, Mesh, MeshStandardMaterial } from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";

/** FRD (x forward, y right, z down) to scene (y up): scene = (x, -z, y). Proper rotation. */
const FRD_TO_SCENE = new Matrix4().set(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1);

interface BoundaryProps {
  children: ReactNode;
  onError: (message: string) => void;
}

/** Keeps a broken model from taking the whole 3D view down; reports the message. */
export class CadErrorBoundary extends Component<BoundaryProps, { failed: boolean }> {
  state = { failed: false };

  static getDerivedStateFromError() {
    return { failed: true };
  }

  componentDidCatch(error: unknown) {
    this.props.onError(error instanceof Error ? error.message : String(error));
  }

  render() {
    return this.state.failed ? null : this.props.children;
  }
}

interface Props {
  url: string;
  opacity?: number;
  onError?: (message: string) => void;
}

/** The airframe meshes exported from the CAD dashboard (glTF, FRD metres relative to the CG). */
export default function CadModel({ url, opacity = 0.35, onError }: Props) {
  const [scene, setScene] = useState<Group | null>(null);
  const material = useMemo(
    () => new MeshStandardMaterial({ color: "#94a3b8", transparent: true, opacity, side: DoubleSide, roughness: 0.8 }),
    [opacity],
  );

  useEffect(() => {
    let cancelled = false;
    setScene(null);
    new GLTFLoader().load(
      url,
      (gltf) => {
        if (cancelled) return;
        let meshes = 0;
        gltf.scene.traverse((o) => {
          if ((o as Mesh).isMesh) {
            (o as Mesh).material = material;
            meshes += 1;
          }
        });
        gltf.scene.matrix.copy(FRD_TO_SCENE);
        gltf.scene.matrixAutoUpdate = false;
        const box = new Box3().setFromObject(gltf.scene);
        // Exposed for tests and debugging: mesh count and FRD bounds of the loaded airframe.
        document.body.dataset.cadMeshes = String(meshes);
        document.body.dataset.cadBounds = `${box.min.toArray().map((v) => v.toFixed(3)).join(",")} .. ${box.max.toArray().map((v) => v.toFixed(3)).join(",")}`;
        setScene(gltf.scene);
      },
      undefined,
      (err) => {
        if (!cancelled) onError?.(err instanceof Error ? err.message : String(err));
      },
    );
    return () => {
      cancelled = true;
    };
  }, [url, material, onError]);

  return scene ? <primitive object={scene} /> : null;
}
