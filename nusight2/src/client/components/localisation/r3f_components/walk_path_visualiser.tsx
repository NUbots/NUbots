import React from "react";
import * as THREE from "three";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";

import { ArrowGeometry } from "./arrow_geometry";

interface WalkPathVisualiserProps {
  Hfd: Matrix4;
  Hfr: Matrix4;
  Hft: Matrix4;
  minAlignRadius: number;
  maxAlignRadius: number;
  minAngleError: number;
  maxAngleError: number;
  angleToFinalHeading: number;
  velocityTarget: Vector3;
}

export const WalkPathVisualiser = (props: WalkPathVisualiserProps) => {
  const {
    Hfd: hfd,
    Hfr: hfr,
    Hft: hft,
    minAlignRadius,
    maxAlignRadius,
    minAngleError,
    maxAngleError,
    angleToFinalHeading,
    velocityTarget,
  } = props;

  if (!hfd || !hfr) {
    return null;
  }

  const rDFf = hfd.decompose().translation;
  const rTFf = hft.decompose().translation;
  const robot_rotation = new THREE.Euler().setFromQuaternion(hft.decompose().rotation.toThree(), "XYZ");
  const target_rotation = new THREE.Euler().setFromQuaternion(hfd.decompose().rotation.toThree(), "XYZ");

  const Rfr = new THREE.Quaternion(
    hfr.decompose().rotation.x,
    hfr.decompose().rotation.y,
    hfr.decompose().rotation.z,
    hfr.decompose().rotation.w,
  );
  const vRf = velocityTarget.toThree().applyQuaternion(Rfr);

  const velocity_direction = Math.atan2(vRf.y, vRf.x);
  const speed = Math.sqrt(vRf.x ** 2 + vRf.y ** 2) * 1.5;

  return (
    <object3D>
      <mesh position={[rDFf?.x, rDFf?.y, 0.005]}>
        <circleGeometry args={[minAlignRadius, 40]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rDFf?.x, rDFf?.y, 0.006]}>
        <circleGeometry args={[maxAlignRadius, 40]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.007]} rotation={[0, 0, target_rotation.z - 0.5 * minAngleError]}>
        <circleGeometry args={[maxAlignRadius, 40, 0, minAngleError]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.008]} rotation={[0, 0, target_rotation.z - 0.5 * maxAngleError]}>
        <circleGeometry args={[maxAlignRadius, 40, 0, maxAngleError]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.009]}>
        <mesh geometry={ArrowGeometry(maxAlignRadius)} rotation={[0, 0, robot_rotation.z]}>
          <meshBasicMaterial color="rgb(255, 255, 255)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.011]}>
        <mesh geometry={ArrowGeometry(speed)} rotation={[0, 0, velocity_direction]}>
          <meshBasicMaterial color="rgb(0, 255, 0)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
      <mesh position={[rDFf?.x, rDFf?.y, 0.011]}>
        <mesh geometry={ArrowGeometry(minAlignRadius)} rotation={[0, 0, robot_rotation.z + angleToFinalHeading]}>
          <meshBasicMaterial color="rgb(255, 0, 0)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
    </object3D>
  );
};
