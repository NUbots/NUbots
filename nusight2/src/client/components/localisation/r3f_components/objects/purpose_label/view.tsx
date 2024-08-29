import { LocalisationRobotModel } from "../../../robot_model";
import { TextBillboard } from "../text_billboard/view";
import React from "react";
import { Matrix4 } from "../../../../../../shared/math/matrix4";

interface PurposeLabelProps {
  Hft: Matrix4;
  player_id: number;
  purpose: string;
  color: string;
  cameraPitch: number;
  cameraYaw: number;
}

export const PurposeLabel: React.FC<PurposeLabelProps> = ({
  Hft,
  player_id,
  purpose,
  color,
  cameraPitch,
  cameraYaw,
}) => {
  const rTFf = Hft.decompose().translation;
  const label = player_id == -1 ? purpose : "N" + player_id + " " + purpose;

  return (
    <TextBillboard
      position={[rTFf?.x, rTFf?.y, rTFf?.z + 0.6]}
      color={color}
      backgroundColor="white"
      text={label}
      cameraPitch={cameraPitch}
      cameraYaw={cameraYaw}
    />
  );
};
