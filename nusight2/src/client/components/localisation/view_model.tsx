import React from "react";
import { observer } from "mobx-react";

import { Vector3 } from "../../../shared/math/vector3";
import { PerspectiveCamera } from "../three/three_fiber";

import { FieldView } from "./field/view_model";
import { LocalisationModel } from "./model";
import { NUgusView } from "./nugus_robot/view_model";
import { SkyboxView } from "./skybox/view_model";

export const LocalisationViewModel = observer(({ model }: { model: LocalisationModel }) => {
  return (
    <object3D>
      <PerspectiveCamera
        args={[75, 1, 0.01, 100]}
        position={model.camera.position.toArray()}
        rotation={[Math.PI / 2 + model.camera.pitch, 0, -Math.PI / 2 + model.camera.yaw, "ZXY"]}
        up={[0, 0, 1]}
      >
        <pointLight color="white" />
      </PerspectiveCamera>
      <FieldView model={model.field} />
      <SkyboxView model={model.skybox} />
      <hemisphereLight args={["#fff", "#fff", 0.6]} />
      {model.robots.map((robotModel) => {
        return robotModel.visible && <NUgusView key={robotModel.id} model={robotModel} />;
      })}
      <FieldLineDots model={model} />
    </object3D>
  );
});
const FieldLineDots = ({ model }: { model: LocalisationModel }) => (
  <>
    {model.robots.map((robot) => (
      <object3D key={robot.id}>
        {robot.fieldLinesDots.rPWw.map((d, i) => {
          return (
            <mesh key={String(i)} position={d.add(new Vector3(0, 0, 0.005)).toArray()}>
              <circleBufferGeometry args={[0.02, 20]} />
              <meshBasicMaterial color="blue" />
            </mesh>
          );
        })}
      </object3D>
    ))}
  </>
);
