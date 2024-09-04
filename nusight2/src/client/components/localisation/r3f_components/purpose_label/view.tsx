import React from "react";
import * as THREE from "three";

import { LocalisationRobotModel } from "../../robot_model";
import { TextBillboard } from "../text_billboard/view";

export const PurposeLabel = ({
  robotModel,
  cameraPitch,
  cameraYaw,
}: {
  robotModel: LocalisationRobotModel;
  cameraPitch: number;
  cameraYaw: number;
}) => {
  const rTFf = robotModel.Hft.decompose().translation;
  const label = robotModel.player_id == -1 ? robotModel.purpose : "N" + robotModel.player_id + " " + robotModel.purpose;

  return (
    <TextBillboard
      position={[rTFf?.x, rTFf?.y, rTFf?.z + 0.6]}
      backgroundColor={robotModel.color}
      textColor="white"
      text={"white"}
      cameraPitch={cameraPitch}
      cameraYaw={cameraYaw}
    />
  );
};
