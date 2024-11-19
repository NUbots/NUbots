import React from "react";
import * as THREE from "three";
import URDFLoader, { URDFRobot } from "urdf-loader";

const nugusUrdfPath = "/robot-models/nugus/robot.urdf";

export const Nugus = () => {
  const robotRef = React.useRef<URDFRobot | null>(null);

  // Load the URDF model only once
  React.useEffect(() => {
    const loader = new URDFLoader();
    loader.load(nugusUrdfPath, (robot: URDFRobot) => {
        // Add the loaded robot to the wrapper
        if (robotRef.current) {
          robotRef.current.add(robot);
        }
      },
    );
  }, [nugusUrdfPath]);

  // Update the material of the robot
  const material = new THREE.MeshStandardMaterial({
    color: "#666666",
    roughness: 0.5,
    metalness: 0.2,
  });
  if (robotRef.current) {
    robotRef.current.traverse((child) => {
      if (child.type === "URDFVisual" && child.children.length > 0) {
        const mesh = child.children[0] as THREE.Mesh;
        mesh.material = material;
      }
    });
  }

  return <object3D ref={robotRef} />;
};
