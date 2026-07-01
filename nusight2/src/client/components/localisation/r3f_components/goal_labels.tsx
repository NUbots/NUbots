import React from "react";
import { observer } from "mobx-react";

import { FieldModel } from "./field/model";
import { TextBillboard } from "./text_billboard";

interface GoalLabelsProps {
  fieldModel: FieldModel;
  cameraPitch: number;
  cameraYaw: number;
}

export const GoalLabels: React.FC<GoalLabelsProps> = observer(({ fieldModel, cameraPitch, cameraYaw }) => {
  const fieldLength = fieldModel.dimensions.fieldLength;
  const goalDepth = fieldModel.dimensions.goalDepth;
  const halfLength = fieldLength * 0.5;

  // In field frame, own goal is always positive x and opponent goal is always negative x.
  // Position labels above the goal center, higher in the air for visibility
  const oppGoalPosition: [number, number, number] = [-halfLength - goalDepth / 2, 0, 1.5];
  const ownGoalPosition: [number, number, number] = [halfLength + goalDepth / 2, 0, 1.5];

  return (
    <object3D>
      {/* Make labels very obvious for debugging */}
      <TextBillboard
        position={oppGoalPosition}
        text="OPP GOAL"
        textColor="white"
        backgroundColor="#ff0000"
        cameraPitch={cameraPitch}
        cameraYaw={cameraYaw}
      />
      <TextBillboard
        position={ownGoalPosition}
        text="OWN GOAL"
        textColor="white"
        backgroundColor="#00ff00"
        cameraPitch={cameraPitch}
        cameraYaw={cameraYaw}
      />
    </object3D>
  );
});
