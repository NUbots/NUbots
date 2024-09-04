import React from "react";

import { Vector3 } from "../../../../../shared/math/vector3";

interface ParticlesProps {
  particles: {
    particle: Vector3[];
  };
}

export const Particles: React.FC<ParticlesProps> = ({ particles }) => (
  <object3D>
    {particles.particle.map((particle, index) => (
      <mesh key={index} position={[particle.x, particle.y, 0.005]}>
        <circleBufferGeometry args={[0.02, 20]} />
        <meshBasicMaterial color="red" />
      </mesh>
    ))}
  </object3D>
);
