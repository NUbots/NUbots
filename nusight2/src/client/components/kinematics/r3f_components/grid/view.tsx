import React from "react";

import { CoordinateLabel } from "../coordinates/view";

export const Grid = ({ gridSize, divisions }: { gridSize: number; divisions: number }) => {
  const labels = [];

  for (let i = 0; i <= divisions; i++) {
    const value = (i - gridSize / 2) / 10;
    const label = value.toFixed(1);

    labels.push(
      // X-axis label
      <CoordinateLabel key={`x-${i}`} text={label} position={[value * 10, -gridSize / 2, gridSize / 2 + 1]} />,
      // Y-axis label
      <CoordinateLabel key={`y-${i}`} text={label} position={[-gridSize / 2 - 0.5, value * 10, gridSize / 2 + 0.5]} />,
      // Z-axis label
      <CoordinateLabel key={`z-${i}`} text={label} position={[gridSize / 2 + 0.5, -gridSize / 2, value * 10]} />,
    );
  }

  return (
    <>
      {/* Grid for X and Z axes */}
      <gridHelper args={[gridSize, divisions, 0x888888, 0x888888]} position={[0, -gridSize / 2, 0]} />

      {/* Grid for X and Y axes */}
      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[0, 0, -gridSize / 2]}
        rotation={[Math.PI / 2, 0, 0]}
      />

      {/* Grid for Y and Z axes */}
      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[-gridSize / 2, 0, 0]}
        rotation={[0, 0, Math.PI / 2]}
      />

      {/* Labels for X, Y, and Z axes */}
      {labels}
    </>
  );
};
