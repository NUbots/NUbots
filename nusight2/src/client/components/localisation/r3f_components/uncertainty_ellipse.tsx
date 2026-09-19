import React from "react";
import * as THREE from "three";

import { Matrix3 } from "../../../../shared/math/matrix3";
import { Vector3 } from "../../../../shared/math/vector3";

interface UncertaintyEllipseProps {
  /** World-to-field pose of the robot; the ellipse is drawn at its translation. */
  position: Vector3;
  /** Covariance of (x, y, theta) from the localisation Field message. */
  covariance: Matrix3;
  /** How many standard deviations the ellipse traces. 3 covers ~99% of the mass in 1D. */
  sigma?: number;
  color?: string;
}

/**
 * The filter's positional uncertainty, drawn on the ground plane.
 *
 * The (x, y) block of the covariance is a 2x2 symmetric matrix, so its eigenvectors give the
 * principal axes of the uncertainty and the square roots of its eigenvalues give the standard
 * deviation along each. Drawing that as an ellipse turns a number nobody reads into something
 * you notice out of the corner of your eye: it inflates on a fall, stays inflated while the
 * robot cannot re-acquire landmarks, and shrinks as vision pulls the estimate back. A pose that
 * is drifting while the ellipse stays small is the signature of a filter that is confidently
 * wrong, which is the failure worth catching early.
 *
 * The closed form for a symmetric 2x2 avoids pulling in an eigensolver:
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

  // A covariance is positive semi-definite, so both eigenvalues are >= 0 -- but a malformed or
  // not-yet-initialised message can arrive as anything, and a NaN here would silently drop the
  // whole scene rather than just this overlay.
  if (!Number.isFinite(a) || !Number.isFinite(b) || !Number.isFinite(c)) {
    return null;
  }

  const mean = (a + c) / 2;
  const halfDiff = (a - c) / 2;
  const radius = Math.sqrt(halfDiff * halfDiff + b * b);
  const major = Math.sqrt(Math.max(mean + radius, 0));
  const minor = Math.sqrt(Math.max(mean - radius, 0));
  const angle = 0.5 * Math.atan2(2 * b, a - c);

  // Degenerate at startup, before any update has run.
  if (major <= 0) {
    return null;
  }

  const curve = new THREE.EllipseCurve(0, 0, sigma * major, sigma * minor, 0, 2 * Math.PI, false, 0);
  const geometry = new THREE.BufferGeometry().setFromPoints(curve.getPoints(64));

  return (
    // Just above the carpet, matching AssociationLines, so it does not z-fight with the field.
    <group position={[position.x, position.y, 0.006]} rotation={[0, 0, angle]}>
      <line>
        <bufferGeometry attach="geometry" {...geometry} />
        <lineBasicMaterial attach="material" color={color} linewidth={2} />
      </line>
    </group>
  );
};
