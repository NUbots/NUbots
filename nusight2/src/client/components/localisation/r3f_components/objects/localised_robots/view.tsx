import React from "react";

import { Vector3 } from "../../../../../../shared/math/vector3";
import { LocalisationModel } from "../../../model";

interface LocalisedRobotsProps {
  id: string;
  rRFf: Vector3[];
}

export const LocalisedRobots = ({ id, rRFf }: LocalisedRobotsProps) => (
  <object3D key={id}>
    {rRFf.map((r, i) => {
      return (
        <mesh key={String(i)} position={r.add(new Vector3(0, 0, 0.4)).toArray()} rotation={[Math.PI / 2, 0, 0]}>
          <cylinderBufferGeometry args={[0.1, 0.1, 0.8, 20]} />
          <meshStandardMaterial color="orange" />
        </mesh>
      );
    })}
  </object3D>
);
