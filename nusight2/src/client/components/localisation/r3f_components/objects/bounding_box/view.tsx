import React, { useEffect, useMemo } from "react";
import * as THREE from "three";

import { LocalisationRobotModel } from "../../../robot_model";

import wallFragmentShader from "./wall.frag";
import wallVertexShader from "./wall.vert";

export const BoundingBox = ({ model }: { model: LocalisationRobotModel }) => {
  if (!model.boundingBox) return null;
  const { minX, maxX, minY, maxY } = model.boundingBox;
  const width = maxX - minX;
  const height = maxY - minY;
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const wallThickness = 0.05;
  const wallHeight = 0.25;
  const solidBottomHeight = 0.001;

  const createWallMaterial = (orientation: number) => {
    return new THREE.ShaderMaterial({
      uniforms: {
        color: { value: new THREE.Color(model.color) },
        wallHeight: { value: wallHeight },
        solidBottomHeight: { value: solidBottomHeight },
        orientation: { value: orientation },
      },
      vertexShader: wallVertexShader,
      fragmentShader: wallFragmentShader,
      transparent: true,
      side: THREE.DoubleSide,
    });
  };

  const verticalWallMaterial = useMemo(() => createWallMaterial(0), []);
  const horizontalWallMaterial = useMemo(() => createWallMaterial(1), []);

  // Update material color when model.color changes
  useEffect(() => {
    const newColor = new THREE.Color(model.color);
    verticalWallMaterial.uniforms.color.value = newColor;
    horizontalWallMaterial.uniforms.color.value = newColor;
  }, [model.color, verticalWallMaterial, horizontalWallMaterial]);

  return (
    <object3D position={[centerX, centerY, wallHeight / 2]}>
      {/* Left wall */}
      <mesh position={[-(width / 2 + wallThickness / 2), 0, 0.009]} rotation={[Math.PI / 2, Math.PI / 2, 0]}>
        <boxGeometry args={[height + wallThickness, wallHeight, wallThickness]} />
        <primitive object={verticalWallMaterial} />
      </mesh>
      {/* Right wall */}
      <mesh position={[width / 2 + wallThickness / 2, 0, 0.009]} rotation={[Math.PI / 2, Math.PI / 2, 0]}>
        <boxGeometry args={[height + wallThickness, wallHeight, wallThickness]} />
        <primitive object={verticalWallMaterial} />
      </mesh>
      {/* Top wall */}
      <mesh position={[0, height / 2 + wallThickness / 2, 0.009]} rotation={[0, 0, 0]}>
        <boxGeometry args={[width + wallThickness * 2, wallThickness, wallHeight]} />
        <primitive object={horizontalWallMaterial} />
      </mesh>
      {/* Bottom wall */}
      <mesh position={[0, -(height / 2 + wallThickness / 2), 0.009]} rotation={[0, 0, 0]}>
        <boxGeometry args={[width + wallThickness * 2, wallThickness, wallHeight]} />
        <primitive object={horizontalWallMaterial} />
      </mesh>
    </object3D>
  );
};
