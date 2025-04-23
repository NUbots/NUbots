import React from "react";
import { Canvas } from "@react-three/fiber";

import { KinematicsRobotModel } from "../robot_model";

import { Axes } from "./axes";
import { CameraControls } from "./camera";
import { Grid } from "./grid";
import { Nugus } from "./nugus";

const RobotComponents: React.FC<{ robot: KinematicsRobotModel }> = ({ robot }) => {
  if (!robot.visible) return null;

  return (
    <object3D key={robot.id} rotation={[-Math.PI / 2, 0, -Math.PI / 2]} scale={[5, 5, 5]}>
      <Nugus model={robot} />
    </object3D>
  );
};

export const CanvasWrapper: React.FC<{ selectedRobot?: KinematicsRobotModel }> = ({ selectedRobot }) => {
  return (
    <Canvas camera={{ position: [10, 10, 10], fov: 60 }}>
      <CameraControls />

      {/* Basic lighting setup */}
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 5]} />

      {/* Render the grid and axes */}
      <Grid gridSize={10} divisions={10} />
      <Axes length={2} />

      {/* Render the selected robot */}
      {selectedRobot && <RobotComponents robot={selectedRobot} />}
    </Canvas>
  );
};
