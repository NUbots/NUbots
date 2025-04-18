import React from "react";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { TrajectoryLine } from "../walk_trajectory/view";

interface WalkTrajectoryHistoryProps {
  trajectories: {
    torso: Matrix4[];
    swing_foot: Matrix4[];
    color: string;
    timestamp: number;
  }[];
  lineWidth?: number;
}

export const WalkTrajectoryHistory: React.FC<WalkTrajectoryHistoryProps> = ({ trajectories, lineWidth = 5 }) => {
  return (
    <group>
      {trajectories.map((trajectory) => {
        const age = Date.now() - trajectory.timestamp;
        const opacity = Math.max(0.1, 1 - age / 10000);

        return (
          <group key={trajectory.timestamp}>
            <TrajectoryLine poses={trajectory.torso} color={trajectory.color} lineWidth={lineWidth} opacity={opacity} />
            <TrajectoryLine
              poses={trajectory.swing_foot}
              color={trajectory.color}
              lineWidth={lineWidth}
              opacity={opacity}
            />
          </group>
        );
      })}
    </group>
  );
};
