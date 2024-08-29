import React from "react";

import { Vector3 } from "../../../../../../shared/math/vector3";
import { LocalisationModel } from "../../../model";

interface GoalsProps {
  rGFf: { bottom: Vector3; top: Vector3 }[];
}

export const Goals = ({ rGFf }: GoalsProps) => (
  <object3D>
    {rGFf.map((goal, i) => {
      return (
        <mesh
          key={String(i)}
          position={goal.bottom.add(new Vector3(0, 0, goal.top.z / 2)).toArray()}
          rotation={[Math.PI / 2, 0, 0]}
        >
          <cylinderBufferGeometry args={[0.05, 0.05, goal.top.z, 20]} />
          <meshStandardMaterial color="magenta" />
        </mesh>
      );
    })}
  </object3D>
);
