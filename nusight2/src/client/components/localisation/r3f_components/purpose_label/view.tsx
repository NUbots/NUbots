import React from "react";
import * as THREE from "three";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { TextBillboard } from "../text_billboard/view";

interface PurposeLabelProps {
  Hft: Matrix4;
  player_id: number;
  purpose: string;
  textColor: string;
  backgroundColor: string;
  cameraPitch: number;
  cameraYaw: number;
}

export const PurposeLabel: React.FC<PurposeLabelProps> = ({
  Hft,
  player_id,
  purpose,
  textColor = "white",
  backgroundColor = "black",
  cameraPitch,
  cameraYaw,
}) => {
  const rTFf = Hft.decompose().translation;
  const label = player_id == -1 ? purpose : "N" + player_id + " " + purpose;

  return (
    <TextBillboard
      position={[rTFf?.x, rTFf?.y, rTFf?.z + 0.6]}
      textColor={textColor}
      backgroundColor={backgroundColor}
      text={label}
      cameraPitch={cameraPitch}
      cameraYaw={cameraYaw}
    />
  );
};
