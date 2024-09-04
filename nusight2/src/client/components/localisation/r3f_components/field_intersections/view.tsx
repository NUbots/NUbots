import React from "react";

import { Vector3 } from "../../../../../shared/math/vector3";
import { FieldIntersection } from "../../robot_model";

interface FieldIntersectionsProps {
  intersections: FieldIntersection[];
}

const IntersectionShape: React.FC<{ intersection: FieldIntersection }> = ({ intersection }) => {
  const basePosition = intersection.position.add(new Vector3(0.1, 0.1, 0)).toArray();

  const commonCircle = (
    <mesh position={intersection.position.add(new Vector3(0, 0, 0.01)).toArray()}>
      <circleBufferGeometry args={[0.04, 20]} />
      <meshBasicMaterial color="red" />
    </mesh>
  );

  switch (intersection.type) {
    case "L_INTERSECTION":
      return (
        <>
          {commonCircle}
          <mesh position={[basePosition[0], basePosition[1] - 0.05, basePosition[2]]}>
            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
            <meshBasicMaterial color="black" />
          </mesh>
          <mesh position={[basePosition[0] - 0.04, basePosition[1], basePosition[2]]}>
            <boxBufferGeometry args={[0.02, 0.1, 0.02]} />
            <meshBasicMaterial color="black" />
          </mesh>
        </>
      );
    case "T_INTERSECTION":
      return (
        <>
          {commonCircle}
          <mesh position={[basePosition[0], basePosition[1] + 0.05, basePosition[2]]}>
            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
            <meshBasicMaterial color="black" />
          </mesh>
          <mesh position={[basePosition[0], basePosition[1], basePosition[2]]}>
            <boxBufferGeometry args={[0.02, 0.1, 0.02]} />
            <meshBasicMaterial color="black" />
          </mesh>
        </>
      );
    case "X_INTERSECTION":
      return (
        <>
          {commonCircle}
          <mesh position={[basePosition[0], basePosition[1], basePosition[2]]} rotation={[0, 0, Math.PI / 4]}>
            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
            <meshBasicMaterial color="black" />
          </mesh>
          <mesh position={[basePosition[0], basePosition[1], basePosition[2]]} rotation={[0, 0, -Math.PI / 4]}>
            <boxBufferGeometry args={[0.1, 0.02, 0.02]} />
            <meshBasicMaterial color="black" />
          </mesh>
        </>
      );
    default:
      return null;
  }
};

export const FieldIntersections: React.FC<FieldIntersectionsProps> = ({ intersections }) => {
  return (
    <>
      {intersections.map((intersection, index) => (
        <object3D key={index}>
          <IntersectionShape intersection={intersection} />
        </object3D>
      ))}
    </>
  );
};
