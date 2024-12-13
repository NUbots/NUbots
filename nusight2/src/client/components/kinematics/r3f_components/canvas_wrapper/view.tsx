import React from 'react';
import { observer } from 'mobx-react';
import { Canvas } from "@react-three/fiber";

import { CameraControls } from "../camera/view";
import { Grid } from "../grid/view";
import { Axes } from "../axes/view";
import { Nugus } from "../nugus/view";
import { KinematicsRobotModel } from "../../robot_model";

const RobotComponents: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  if (!robot.visible) return null;

  return (
    <object3D key={robot.id} rotation={[-Math.PI / 2, 0, -Math.PI / 2]} scale={[5, 5, 5]}>
      <Nugus model={robot} />
    </object3D>
  );
});

export const CanvasWrapper: React.FC<{ selectedRobot?: KinematicsRobotModel }> = ({ selectedRobot }) => (
  <Canvas camera={{ position: [10, 10, 10], fov: 60 }} className="w-full h-full">
    <CameraControls />

    <ambientLight intensity={0.5} />
    <directionalLight position={[10, 10, 5]} />

    <Grid gridSize={10} divisions={10} />
    <Axes length={2} />

    {selectedRobot && <RobotComponents robot={selectedRobot} />}
  </Canvas>
);
