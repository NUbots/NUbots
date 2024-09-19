import React from "react";

import { Vector3 } from "../../../../../shared/math/vector3";
import { LocalisationModel } from "../../model";

export const LocalisedRobots = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map(
      (robot) =>
        robot.visible &&
        robot.robots && (
          <object3D key={robot.id}>
            {robot.rRFf.map((r, i) => {
              return (
                <mesh key={String(i)} position={r.add(new Vector3(0, 0, 0.4)).toArray()} rotation={[Math.PI / 2, 0, 0]}>
                  <cylinderBufferGeometry args={[0.1, 0.1, 0.8, 20]} />
                  <meshStandardMaterial color="orange" />
                </mesh>
              );
            })}
          </object3D>
        ),
    )}
  </>
);
