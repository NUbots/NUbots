import { useMemo } from "react";
import * as THREE from "three";

export const ArrowGeometry = (length: number): THREE.ShapeGeometry => {
  const arrowShape = useMemo(() => {
    const arrowShape = new THREE.Shape();

    arrowShape.moveTo(0, -0.01);
    arrowShape.lineTo(0, 0.01);
    arrowShape.lineTo(length * 0.7, 0.01);
    arrowShape.lineTo(length * 0.7, 0.02);
    arrowShape.lineTo(length, 0);
    arrowShape.lineTo(length * 0.7, -0.02);
    arrowShape.lineTo(length * 0.7, -0.01);
    arrowShape.lineTo(0, -0.01);

    return arrowShape;
  }, [length]);

  return useMemo(() => new THREE.ShapeGeometry(arrowShape), [arrowShape]);
};
