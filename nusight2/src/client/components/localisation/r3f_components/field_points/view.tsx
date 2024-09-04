import React, { useMemo, useRef, useEffect } from "react";
import * as THREE from "three";
import { Vector3 } from "../../../../../shared/math/vector3";

interface FieldPoints {
  points: Vector3[];
  color?: string;
  size?: number;
}

export const FieldPoints: React.FC<FieldPoints> = ({ points, color = "blue", size = 0.02 }) => {
  const meshRef = useRef<THREE.InstancedMesh>(null);

  const pointGeometry = useMemo(() => new THREE.CircleGeometry(size, 20), [size]);
  const pointMaterial = useMemo(() => new THREE.MeshBasicMaterial({ color }), [color]);

  useEffect(() => {
    if (meshRef.current) {
      const matrix = new THREE.Matrix4();
      points.forEach((point, index) => {
        matrix.setPosition(point.x, point.y, point.z + 0.005);
        meshRef.current?.setMatrixAt(index, matrix);
      });
      meshRef.current.instanceMatrix.needsUpdate = true;
    }
  }, [points]);

  return (
    <instancedMesh
      ref={meshRef}
      args={[pointGeometry, pointMaterial, points.length]}
    />
  );
};
