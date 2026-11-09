import React from "react";

import { Matrix4 } from "../../../../shared/math/matrix4";

import { TextBillboard } from "./text_billboard";

interface TimeToBallLabelProps {
  Hft: Matrix4;
  playerId: number;
  time: number;
  backgroundColor: string;
  cameraPitch: number;
  cameraYaw: number;
}

export const TimeToBallLabel: React.FC<TimeToBallLabelProps> = ({
  Hft,
  playerId,
  time,
  backgroundColor,
  cameraPitch,
  cameraYaw,
}) => {
  const rTFf = Hft.decompose().translation;
  const label = (playerId === -1 ? "Self" : "N" + playerId) + `: ${time.toFixed(1)}s`;

  return (
    <TextBillboard
      // Stacked directly above the purpose label (which sits at z + 0.6) so the two read as a group.
      position={[rTFf?.x, rTFf?.y, rTFf?.z + 0.9]}
      textColor="white"
      backgroundColor={backgroundColor}
      text={label}
      cameraPitch={cameraPitch}
      cameraYaw={cameraYaw}
    />
  );
};
