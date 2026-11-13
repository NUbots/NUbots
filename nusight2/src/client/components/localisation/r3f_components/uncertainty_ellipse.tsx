import React from "react";
import * as THREE from "three";

import { Matrix3 } from "../../../../shared/math/matrix3";
import { Vector3 } from "../../../../shared/math/vector3";

interface UncertaintyEllipseProps {
  /** Position the ellipse is drawn at */
  position: Vector3;
  /** Covariance of (x, y, theta) from the localisation Field message */
  covariance: Matrix3;
  /** How many standard deviations the ellipse traces */
  sigma?: number;
  color?: string;
}

/**
 * The localisation's positional uncertainty, drawn on the ground plane as an ellipse along the
 * principal axes of the (x, y) covariance, from the closed form for a symmetric 2x2:
 *   lambda = (a + c)/2 +- sqrt(((a - c)/2)^2 + b^2)
 *   angle  = 0.5 * atan2(2b, a - c)
 */
export const UncertaintyEllipse: React.FC<UncertaintyEllipseProps> = ({
  position,
  covariance,
  sigma = 3,
  color = "orange",
}) => {
  const a = covariance.x.x;
  const b = covariance.x.y;
  const c = covariance.y.y;

  // Draw nothing for a malformed covariance, as a NaN would drop the whole scene
  if (!Number.isFinite(a) || !Number.isFinite(b) || !Number.isFinite(c)) {
    return null;
  }

  const mean = (a + c) / 2;
  const halfDiff = (a - c) / 2;
  const radius = Math.sqrt(halfDiff * halfDiff + b * b);
  const major = Math.sqrt(Math.max(mean + radius, 0));
  const minor = Math.sqrt(Math.max(mean - radius, 0));
  const angle = 0.5 * Math.atan2(2 * b, a - c);

  // Degenerate before any update has run
  if (major <= 0) {
    return null;
  }

  const curve = new THREE.EllipseCurve(0, 0, sigma * major, sigma * minor, 0, 2 * Math.PI, false, 0);
  const geometry = new THREE.BufferGeometry().setFromPoints(curve.getPoints(64));

  return (
    // Just above the carpet, so it does not z-fight with the field
    <group position={[position.x, position.y, 0.006]} rotation={[0, 0, angle]}>
      <line>
        <bufferGeometry attach="geometry" {...geometry} />
        <lineBasicMaterial attach="material" color={color} linewidth={2} />
      </line>
    </group>
  );
};
