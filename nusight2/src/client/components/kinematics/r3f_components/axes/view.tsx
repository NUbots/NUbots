import React from 'react';
import { Euler } from 'three';
import { CoordinateLabel } from '../coordinates/view';

const Arrow = ({ dir, color, length, thickness }: {
  dir: [number, number, number];
  color: number;
  length: number;
  thickness: number;
}) => {
  const shaftLength = length * 0.90;
  const headLength = length * 0.1;

  // Determine rotation for each axis direction
  const rotation = (() => {
    if (dir[0] === 1) return new Euler(0, 0, -Math.PI / 2);
    if (dir[1] === 1) return new Euler(0, 0, 0);
    if (dir[2] === 1) return new Euler(Math.PI / 2, 0, 0);
    return new Euler(0, 0, 0);
  })();

  return (
    <group rotation={rotation}>
      {/* Arrow shaft (cylinder) */}
      <mesh position={[0, shaftLength / 2, 0]}>
        <cylinderGeometry
          args={[thickness, thickness, shaftLength, 32]}
        />
        <meshBasicMaterial color={color} />
      </mesh>

      {/* Arrow head (cone) */}
      <mesh position={[0, shaftLength + headLength / 2, 0]}>
        <coneGeometry
          args={[thickness * 2, headLength, 32]}
        />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
};

export const Axes = ({ length }: { length: number }) => (
  <>
    <CoordinateLabel text="X" position={[length + 1, 0, -0.25]} />
    <Arrow dir={[1, 0, 0]} color={0xff0000} length={length + 1} thickness={0.03} />

    <CoordinateLabel text="Y" position={[0.25, length + 1, 0]} />
    <Arrow dir={[0, 1, 0]} color={0x00ff00} length={length + 1} thickness={0.03} />

    <CoordinateLabel text="Z" position={[-0.25, 0, length + 1]} />
    <Arrow dir={[0, 0, 1]} color={0x0000ff} length={length + 1} thickness={0.03} />
  </>
);
