import React from "react";
import { observer } from "mobx-react";

import { KinematicsRobotModel } from "../../robot_model";

export const JointDataDisplay: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  return (
    <div className="p-4 border border-black dark:border-white rounded-lg w-full">
      <h3 className="text-xl font-semibold mb-4 pb-2">Joint Angles</h3>
      <ul className="space-y-2">
        {Object.entries(robot.motors).map(([jointName, motor]) => {
          const formattedLabel = jointName
            .replace(/([a-z])([A-Z])/g, "$1 $2")
            .replace(/^./, (match) => match.toUpperCase());
          return (
            <li
              key={jointName}
              className="flex justify-between items-center p-2 border-black dark:border-white border-b"
            >
              <span className="font-medium">{formattedLabel}</span>
              <span className="text-right">{motor.angle.toFixed(2)}°</span>
            </li>
          );
        })}
      </ul>
    </div>
  );
});
