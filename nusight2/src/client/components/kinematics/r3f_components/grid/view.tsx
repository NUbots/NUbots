import React from 'react';
import { CoordinateLabel } from '../coordinates/view';

export const Grid = ({ gridSize, divisions }: { gridSize: number; divisions: number }) => {
  const labels = [];

  for (let i = 0; i <= divisions; i++) {
    const value = (i - gridSize / 2) / 10;

    labels.push(
      // X-axis labels
      <React.Fragment key={`x-${i}`}>
        <CoordinateLabel text={value.toFixed(1)} position={[value * 10, -gridSize / 2, (gridSize / 2) + 1]} />
      </React.Fragment>,

      // Y-axis labels
      <React.Fragment key={`y-${i}`}>
        <CoordinateLabel text={value.toFixed(1)} position={[(-gridSize / 2) - 0.5, value * 10, (gridSize / 2) + 0.5]} />
      </React.Fragment>,

      // Z-axis labels
      <React.Fragment key={`z-${i}`}>
        <CoordinateLabel text={value.toFixed(1)} position={[(gridSize / 2) + 0.5, -gridSize / 2, value * 10]} />
      </React.Fragment>,
    );
  }

  return (
    <>
      {/* Grid for X and Z axes */}
      <gridHelper
        args={[gridSize, divisions, 0x888888, 0x888888]}
        position={[0, -gridSize / 2, 0]}
      />

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

      {/* Labels */}
      {labels}
    </>
  );
};
