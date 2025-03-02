import React from "react";
import * as THREE from "three";
import { Line } from "../../robot_model"; // Adjust this path to the correct tapestry of your type structure

interface AssociationLinesProps {
  lines?: Line[];
}

export const AssociationLines: React.FC<AssociationLinesProps> = ({ lines }) => {
  return (
    <>
      <group>
        {lines.map((line, index) => {
          const start = new THREE.Vector3(line.start.x, line.start.y, 0.005);
          const end = new THREE.Vector3(line.end.x, line.end.y, 0.005);

          const geometry = new THREE.BufferGeometry().setFromPoints([start, end]);

          return (
            <line key={index}>
              <bufferGeometry attach="geometry" {...geometry} />
              <lineBasicMaterial attach="material" color="red" linewidth={4} />
            </line>
          );
        })}
      </group>
    </>
  );
};
