import React, { useMemo } from "react";
import * as THREE from "three";

import { useTextGeometry } from "./text_geometry_helper";

const TEXT_OPACITY = 1;
const TEXT_BACKDROP_OPACITY = 0.5;

const textBackdropGeometry = (width: number, height: number) => {
  const shape = new THREE.Shape();
  width += 0.1;
  height += 0.1;
  const radius = 0.05;
  const x = width * -0.5;
  const y = height * -0.5;

  shape.moveTo(x, y + radius);
  shape.lineTo(x, y + height - radius);
  shape.quadraticCurveTo(x, y + height, x + radius, y + height);
  shape.lineTo(x + width - radius, y + height);
  shape.quadraticCurveTo(x + width, y + height, x + width, y + height - radius);
  shape.lineTo(x + width, y + radius);
  shape.quadraticCurveTo(x + width, y, x + width - radius, y);
  shape.lineTo(x + radius, y);
  shape.quadraticCurveTo(x, y, x, y + radius);

  return new THREE.ShapeGeometry(shape);
};

export const TextBillboard = ({
  position,
  text,
  textColor,
  backgroundColor,
  cameraPitch,
  cameraYaw,
}: {
  position: [number, number, number];
  text: string;
  textColor: string;
  backgroundColor: string;
  cameraPitch: number;
  cameraYaw: number;
}) => {
  const textGeometry = useTextGeometry(text);

  // Always define fallback dimensions
  const textWidth = textGeometry?.boundingBox
    ? textGeometry.boundingBox.max.x - textGeometry.boundingBox.min.x
    : 0;
  const textHeight = textGeometry?.boundingBox
    ? textGeometry.boundingBox.max.y - textGeometry.boundingBox.min.y
    : 0;

  // ✅ Always call useMemo
  const backdropGeometry = useMemo(
    () => textBackdropGeometry(textWidth, textHeight),
    [textWidth, textHeight]
  );

  // Early return to avoid rendering broken mesh
  if (!textGeometry) return null;

  return (
    <object3D position={position} rotation={[Math.PI / 2 + cameraPitch, 0, -Math.PI / 2 + cameraYaw, "ZXY"]}>
      <mesh position={[0, 0, -0.01]} geometry={backdropGeometry}>
        <meshBasicMaterial color={backgroundColor} transparent opacity={TEXT_BACKDROP_OPACITY} />
      </mesh>
      <mesh geometry={textGeometry}>
        <meshBasicMaterial color={textColor} transparent opacity={TEXT_OPACITY} />
      </mesh>
    </object3D>
  );
};
