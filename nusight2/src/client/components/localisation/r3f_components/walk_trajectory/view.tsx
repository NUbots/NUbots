import React from "react";
import * as THREE from "three";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { Vector3 } from "../../../../../shared/math/vector3";

interface WalkTrajectoryProps {
  torso_trajectory: Matrix4[];
  swing_foot_trajectory: Matrix4[];
  color?: string;
  lineWidth?: number;
}

export const WalkTrajectory: React.FC<WalkTrajectoryProps> = ({
  torso_trajectory,
  swing_foot_trajectory,
  color = "#ff0000",
  lineWidth = 5,
}) => {
  // Create points for torso trajectory line
  const torsoPoints = torso_trajectory.map((pose) => {
    const position = new Vector3(pose.t.x, pose.t.y, pose.t.z);
    return new THREE.Vector3(position.x, position.y, position.z);
  });

  // Create points for swing foot trajectory line
  const swingPoints = swing_foot_trajectory.map((pose) => {
    const position = new Vector3(pose.t.x, pose.t.y, pose.t.z);
    return new THREE.Vector3(position.x, position.y, position.z);
  });

  // Create geometry for both lines
  const torsoGeometry = new THREE.BufferGeometry().setFromPoints(torsoPoints);
  const swingGeometry = new THREE.BufferGeometry().setFromPoints(swingPoints);

  // Create materials
  const torsoMaterial = new THREE.LineBasicMaterial({ color: "#ff0000", linewidth: lineWidth });
  const swingMaterial = new THREE.LineBasicMaterial({ color: "#ff0000", linewidth: lineWidth });

  return (
    <group>
      <line geometry={torsoGeometry} material={torsoMaterial} />
      <line geometry={swingGeometry} material={swingMaterial} />
    </group>
  );
};
