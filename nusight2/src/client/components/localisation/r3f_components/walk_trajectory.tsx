import React from "react";
import * as THREE from "three";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";

interface TrajectoryLineProps {
  poses: Matrix4[];
  color: string;
  lineWidth?: number;
  opacity?: number;
}

export const TrajectoryLine: React.FC<TrajectoryLineProps> = ({ poses, color, lineWidth = 2, opacity = 1.0 }) => {
  // Create points for trajectory line
  const points = poses.map((pose) => {
    const position = new Vector3(pose.t.x, pose.t.y, pose.t.z);
    return new THREE.Vector3(position.x, position.y, position.z);
  });

  // Create geometry
  const geometry = new THREE.BufferGeometry().setFromPoints(points);

  // Create material
  const material = new THREE.LineBasicMaterial({
    color: new THREE.Color(color),
    linewidth: lineWidth,
    transparent: opacity < 1.0,
    opacity: opacity,
  });
  // @ts-ignore
  return <line geometry={geometry} material={material} />;
};

interface WalkTrajectoryProps {
  torsoTrajectory: Matrix4[];
  swingFootTrajectory: Matrix4[];
  color?: string;
  lineWidth?: number;
}

export const WalkTrajectory: React.FC<WalkTrajectoryProps> = ({
  torsoTrajectory,
  swingFootTrajectory,
  color = "#ffa500",
  lineWidth = 5,
}) => {
  return (
    <group>
      <TrajectoryLine poses={torsoTrajectory} color={color} lineWidth={lineWidth} />
      <TrajectoryLine poses={swingFootTrajectory} color={color} lineWidth={lineWidth} />
    </group>
  );
};
