import React from "react";
import { Canvas } from "@react-three/fiber";
import { observer } from "mobx-react";

import { KinematicsRobotModel } from "../../robot_model";
import { Axes } from "../axes/view";
import { CameraControls } from "../camera/view";
import { Grid } from "../grid/view";
import { Nugus } from "../nugus/view";

const RobotComponents: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  if (!robot.visible) return null;

  const changingAttribute = robot.Htw; // read a mobx observable property to trigger re-render

  return (
    <object3D key={robot.id} rotation={[-Math.PI / 2, 0, -Math.PI / 2]} scale={[5, 5, 5]}>
      <Nugus model={robot} />
    </object3D>
  );
});

export const CanvasWrapper: React.FC<{ selectedRobot?: KinematicsRobotModel }> = observer(({ selectedRobot }) => {
  return (
    <Canvas camera={{ position: [10, 10, 10], fov: 60 }} className="w-full h-full">
      <CameraControls />

      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 5]} />

      <Grid gridSize={10} divisions={10} />
      <Axes length={2} />

      <RobotComponents robot={selectedRobot} />
    </Canvas>
  );
});
