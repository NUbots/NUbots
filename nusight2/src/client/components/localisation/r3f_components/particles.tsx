import React, { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";

import { Vector3 } from "../../../../shared/math/vector3";
import { ArrowGeometry } from "./arrow_geometry";

interface ParticlesProps {
  particles: Vector3[];
  color?: string;
  length?: number;
}

export const Particles: React.FC<ParticlesProps> = ({ particles, color = "blue", length = 0.05 }) => {
  const meshRef = useRef<THREE.InstancedMesh>(null);
  
  const arrowGeometry = ArrowGeometry(length);
  const material = useMemo(() => new THREE.MeshBasicMaterial({ color }), [color]);

  useEffect(() => {
    if (meshRef.current) {
      const matrix = new THREE.Matrix4();
      const quaternion = new THREE.Quaternion();
      const position = new THREE.Vector3();
      const scale = new THREE.Vector3(1, 1, 1);

      particles.forEach((particle, index) => {
        position.set(particle.x, particle.y, 0.005);
        // Particle.z is theta (heading)
        quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), particle.z);
        matrix.compose(position, quaternion, scale);
        meshRef.current?.setMatrixAt(index, matrix);
      });
      meshRef.current.instanceMatrix.needsUpdate = true;
    }
  }, [particles]);

  return <instancedMesh ref={meshRef} args={[arrowGeometry, material, particles.length]} />;
};
