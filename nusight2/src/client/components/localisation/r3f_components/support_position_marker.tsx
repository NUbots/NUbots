import React from "react";

import { Vector2 } from "../../../../shared/math/vector2";

interface SupportPositionMarkerProps {
  position: Vector2;
  color: string;
  radius?: number;
}

export const SupportPositionMarker: React.FC<SupportPositionMarkerProps> = ({ position, color, radius = 0.1 }) => (
  <mesh position={[position.x, position.y, 0.01]}>
    <circleGeometry args={[radius, 32]} />
    <meshBasicMaterial color={color} />
  </mesh>
);
