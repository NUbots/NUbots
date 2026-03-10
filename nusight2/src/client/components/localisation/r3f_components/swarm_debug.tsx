import React from "react";
import * as THREE from "three";

import { Matrix2 } from "../../../../shared/math/matrix2";
import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";

import { ConfidenceEllipse } from "./confidence_ellipse";

interface SwarmTeammateMarkersProps {
  /** Raw self-reported positions of teammates in field space (from RoboCup broadcasts) */
  positions: Vector3[];
  /** Localisation costs for each teammate (parallel to positions) */
  costs: number[];
}

/**
 * Renders small diamond-shaped markers at each teammate's self-reported field position.
 * A small cost ring beneath the diamond shows how confident that teammate's localisation is.
 */
export const SwarmTeammateMarkers: React.FC<SwarmTeammateMarkersProps> = ({ positions, costs }) => {
  return (
    <group>
      {positions.map((pos, index) => {
        // Draw a small flat diamond on the field plane to distinguish from UKF cylinders
        const s = 0.12; // half-size of diamond
        const z = 0.01;
        const pts = [
          new THREE.Vector3(pos.x, pos.y + s, z),
          new THREE.Vector3(pos.x + s, pos.y, z),
          new THREE.Vector3(pos.x, pos.y - s, z),
          new THREE.Vector3(pos.x - s, pos.y, z),
          new THREE.Vector3(pos.x, pos.y + s, z), // close the diamond
        ];
        const geometry = new THREE.BufferGeometry().setFromPoints(pts);
        const cost = costs[index] ?? 1.0;
        // Simple isotropic cost ring: radius proportional to sqrt(cost)
        const sigma = Math.sqrt(Math.max(0, cost));
        const variance = sigma * sigma;
        const cov = new Matrix2(new Vector2(variance, 0), new Vector2(0, variance));
        return (
          <group key={index}>
            <line>
              <bufferGeometry attach="geometry" {...geometry} />
              <lineBasicMaterial attach="material" color="#ffff00" linewidth={3} />
            </line>
            {cost > 0 && (
              <ConfidenceEllipse center={pos} covariance={cov} color="#ffff44" scale={1.0} segments={32} />
            )}
          </group>
        );
      })}
    </group>
  );
};

interface SwarmOpponentTracksProps {
  /** Confirmed KF-tracked opponent positions + covariances in field space */
  opponents: { position: Vector3; covariance: Matrix2 }[];
}

/**
 * Renders orange confidence ellipses + centre dots for each confirmed opponent KF track.
 * Ellipse size reflects the KF uncertainty — small = confident track, large = uncertain.
 */
export const SwarmOpponentTracks: React.FC<SwarmOpponentTracksProps> = ({ opponents }) => {
  return (
    <group>
      {opponents.map((opp, index) => {
        const z = 0.005;
        // Small dot at track centre
        const r = 0.08;
        const dotPts: THREE.Vector3[] = [];
        for (let i = 0; i <= 16; i++) {
          const t = (i / 16) * 2 * Math.PI;
          dotPts.push(new THREE.Vector3(opp.position.x + r * Math.cos(t), opp.position.y + r * Math.sin(t), z));
        }
        const dotGeom = new THREE.BufferGeometry().setFromPoints(dotPts);
        return (
          <group key={index}>
            <line>
              <bufferGeometry attach="geometry" {...dotGeom} />
              <lineBasicMaterial attach="material" color="#ff8800" linewidth={2} />
            </line>
            <ConfidenceEllipse center={opp.position} covariance={opp.covariance} color="#ff8800" scale={1.0} segments={48} />
          </group>
        );
      })}
    </group>
  );
};

interface SwarmBallMarkerProps {
  /** Fused ball position in field space */
  position: Vector3;
  /** 2x2 ball position covariance [m²] */
  covariance: Matrix2;
  /** Whether the swarm has a confirmed ball estimate */
  seen: boolean;
}

/**
 * Renders a yellow confidence ellipse + dot for the swarm-fused ball position.
 * Only shown when the swarm has at least one recent ball report.
 */
export const SwarmBallMarker: React.FC<SwarmBallMarkerProps> = ({ position, covariance, seen }) => {
  if (!seen) return null;
  const z = 0.005;
  const r = 0.1;
  const dotPts: THREE.Vector3[] = [];
  for (let i = 0; i <= 16; i++) {
    const t = (i / 16) * 2 * Math.PI;
    dotPts.push(new THREE.Vector3(position.x + r * Math.cos(t), position.y + r * Math.sin(t), z));
  }
  const dotGeom = new THREE.BufferGeometry().setFromPoints(dotPts);
  return (
    <group>
      <line>
        <bufferGeometry attach="geometry" {...dotGeom} />
        <lineBasicMaterial attach="material" color="#ffdd00" linewidth={2} />
      </line>
      <ConfidenceEllipse center={position} covariance={covariance} color="#ffdd00" scale={1.0} segments={48} />
    </group>
  );
};
