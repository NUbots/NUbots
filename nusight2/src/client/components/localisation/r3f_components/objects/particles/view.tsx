import React from "react";
import { Vector3 } from "three";

import { LocalisationModel } from "../../../model";

export const Particles = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map(
      (robot) =>
        robot.visible && (
          <object3D key={robot.id}>
            {robot.particles.particle.map((particle, i) => {
              return (
                <mesh key={String(i)} position={new Vector3(particle.x, particle.y, 0.005).toArray()}>
                  <circleBufferGeometry args={[0.02, 20]} />
                  <meshBasicMaterial color="red" />
                </mesh>
              );
            })}
          </object3D>
        ),
    )}
  </>
);
