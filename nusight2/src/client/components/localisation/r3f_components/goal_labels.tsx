import React from "react";
import { observer } from "mobx-react";

import { FieldModel } from "./field/model";
import { TextBillboard } from "./text_billboard";

interface GoalLabelsProps {
  fieldModel: FieldModel;
  teamColour: "red" | "blue";
  cameraPitch: number;
  cameraYaw: number;
}

export const GoalLabels: React.FC<GoalLabelsProps> = observer(({ fieldModel, teamColour, cameraPitch, cameraYaw }) => {
  const fieldLength = fieldModel.dimensions.fieldLength;
  const goalDepth = fieldModel.dimensions.goalDepth;
  const halfLength = fieldLength * 0.5;

  // Blue goal is at negative x, Yellow goal is at positive x
  // Position labels above the goal center, higher in the air for visibility
  const blueGoalPosition: [number, number, number] = [-halfLength - goalDepth / 2, 0, 1.5];
  const yellowGoalPosition: [number, number, number] = [halfLength + goalDepth / 2, 0, 1.5];

  // Determine goal labels based on team color
  // Blue goal is at negative x, Yellow goal is at positive x
  const blueGoalLabel = teamColour === "blue" ? "OWN" : "OPP";
  const yellowGoalLabel = teamColour === "blue" ? "OPP" : "OWN";

  return (
    <object3D>
      {/* Make labels very obvious for debugging */}
      <TextBillboard
        position={blueGoalPosition}
        text={`${blueGoalLabel} GOAL`}
        textColor="white"
        backgroundColor="#ff0000"
        cameraPitch={cameraPitch}
        cameraYaw={cameraYaw}
      />
      <TextBillboard
        position={yellowGoalPosition}
        text={`${yellowGoalLabel} GOAL`}
        textColor="white"
        backgroundColor="#00ff00"
        cameraPitch={cameraPitch}
        cameraYaw={cameraYaw}
      />
    </object3D>
  );
});
