import React from "react";

import { Matrix4 } from "../../../../shared/math/matrix4";

import { TextBillboard } from "./text_billboard";

interface PurposeLabelProps {
  Hft: Matrix4;
  playerId: number;
  purpose: string;
  textColor?: string;
  backgroundColor: string;
  cameraPitch: number;
  cameraYaw: number;
}

export const PurposeLabel: React.FC<PurposeLabelProps> = ({
  Hft,
  playerId,
  purpose,
  textColor = "white",
  backgroundColor = "black",
  cameraPitch,
  cameraYaw,
}) => {
  const rTFf = Hft.decompose().translation;
  const label = playerId == -1 ? purpose : "N" + playerId + " " + purpose;

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
