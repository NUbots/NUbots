import React, { useMemo } from "react";
import * as THREE from "three";

import { Matrix2 } from "../../../../shared/math/matrix2";
import { Vector3 } from "../../../../shared/math/vector3";

interface ConfidenceEllipseProps {
  /** Robot position in field space (x, y used; z ignored) */
  center: Vector3;
  /** 2x2 covariance of field line observations in field space */
  covariance: Matrix2;
  /** Scale factor applied to the ellipse semi-axes (default 1.0) */
  scale?: number;
  /** Colour of the ellipse ring */
  color?: string;
  /** Number of segments around the ellipse (default 64) */
  segments?: number;
}

/**
 * Draws a confidence ellipse on the field plane derived from the 2x2 covariance
 * of field line observations projected to field space.
 *
 * The ellipse is computed via the closed-form eigendecomposition of a 2x2
 * symmetric matrix:
 *   Given Σ = [[a, b], [b, c]]:
 *     mean  = (a + c) / 2
 *     delta = sqrt(((a - c) / 2)^2 + b^2)
 *     λ1    = mean + delta   (larger eigenvalue → major axis)
 *     λ2    = mean - delta   (smaller eigenvalue → minor axis)
 *     angle = atan2(λ1 - a, b)  (rotation of major axis)
 *
 * Semi-axes are scale * sqrt(eigenvalue), so the ellipse encloses roughly the
 * 1-sigma region of the observation distribution.
 *
 * Interpretation:
 *   - A thin, elongated ellipse means observations are clustered along one direction
 *     → poor localisation constraint in the perpendicular direction.
 *   - A round ellipse means observations are spread in all directions
 *     → well-constrained localisation.
 */
export const ConfidenceEllipse: React.FC<ConfidenceEllipseProps> = ({
  center,
  covariance,
  scale = 1.0,
  color = "#00ffff",
  segments = 64,
}) => {
  const geometry = useMemo(() => {
    // Unpack the 2x2 symmetric covariance.
    // Matrix2 stores column vectors: x = col0, y = col1.
    // So cov = [[x.x, y.x], [x.y, y.y]]
    const a = covariance.x.x; // Σ[0,0]
    const b = covariance.y.x; // Σ[0,1] = Σ[1,0]
    const c = covariance.y.y; // Σ[1,1]

    // Closed-form 2x2 symmetric eigendecomposition
    const mean = (a + c) / 2;
    const delta = Math.sqrt(Math.max(0, ((a - c) / 2) ** 2 + b * b));
    const lambda1 = mean + delta; // larger eigenvalue
    const lambda2 = mean - delta; // smaller eigenvalue

    // Semi-axes (1-sigma), with a minimum to avoid degenerate geometry
    const MIN_AXIS = 0.05;
    const semiMajor = scale * Math.max(MIN_AXIS, Math.sqrt(Math.max(0, lambda1)));
    const semiMinor = scale * Math.max(MIN_AXIS, Math.sqrt(Math.max(0, lambda2)));

    // Rotation angle of the major axis (angle from x-axis to first eigenvector)
    const angle = Math.abs(b) < 1e-10 && a >= c ? 0 : Math.atan2(lambda1 - a, b);

    // Build ellipse as a set of line segments on the z=0 plane
    const points: THREE.Vector3[] = [];
    for (let i = 0; i <= segments; i++) {
      const t = (i / segments) * 2 * Math.PI;
      // Ellipse in local frame
      const ex = semiMajor * Math.cos(t);
      const ey = semiMinor * Math.sin(t);
      // Rotate by angle and translate to center
      const wx = center.x + ex * Math.cos(angle) - ey * Math.sin(angle);
      const wy = center.y + ex * Math.sin(angle) + ey * Math.cos(angle);
      points.push(new THREE.Vector3(wx, wy, 0.003)); // slightly above field
    }

    return new THREE.BufferGeometry().setFromPoints(points);
  }, [center, covariance, scale, segments]);

  return (
    <line>
      <bufferGeometry attach="geometry" {...geometry} />
      <lineBasicMaterial attach="material" color={color} linewidth={2} transparent opacity={0.7} />
    </line>
  );
};
