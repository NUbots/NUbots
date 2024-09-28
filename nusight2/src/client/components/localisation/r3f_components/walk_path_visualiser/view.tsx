import React from "react";
import * as THREE from "three";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { Vector3 } from "../../../../../shared/math/vector3";
import { ArrowGeometry } from "../arrow_geometry/ArrowGeometry";

interface WalkPathVisualiserProps {
  Hfd: Matrix4;
  Hfr: Matrix4;
  Hft: Matrix4;
  min_align_radius: number;
  max_align_radius: number;
  min_angle_error: number;
  max_angle_error: number;
  angle_to_final_heading: number;
  velocity_target: Vector3;
}

export const WalkPathVisualiser = (props: WalkPathVisualiserProps) => {
  const {
    Hfd: hfd,
    Hfr: hfr,
    Hft: hft,
    min_align_radius,
    max_align_radius,
    min_angle_error,
    max_angle_error,
    angle_to_final_heading,
    velocity_target,
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
  const vRf = velocity_target.toThree().applyQuaternion(Rfr);

  const velocity_direction = Math.atan2(vRf.y, vRf.x);
  const speed = Math.sqrt(vRf.x ** 2 + vRf.y ** 2) * 1.5;

  return (
    <object3D>
      <mesh position={[rDFf?.x, rDFf?.y, 0.005]}>
        <circleBufferGeometry args={[min_align_radius, 40]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rDFf?.x, rDFf?.y, 0.006]}>
        <circleBufferGeometry args={[max_align_radius, 40]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.007]} rotation={[0, 0, target_rotation.z - 0.5 * min_angle_error]}>
        <circleBufferGeometry args={[max_align_radius, 40, 0, min_angle_error]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.008]} rotation={[0, 0, target_rotation.z - 0.5 * max_angle_error]}>
        <circleBufferGeometry args={[max_align_radius, 40, 0, max_angle_error]} />
        <meshBasicMaterial color="rgb(0, 100, 100)" opacity={0.25} transparent={true} />
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.009]}>
        <mesh geometry={ArrowGeometry(max_align_radius)} rotation={[0, 0, robot_rotation.z]}>
          <meshBasicMaterial color="rgb(255, 255, 255)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
      <mesh position={[rTFf?.x, rTFf?.y, 0.011]}>
        <mesh geometry={ArrowGeometry(speed)} rotation={[0, 0, velocity_direction]}>
          <meshBasicMaterial color="rgb(0, 255, 0)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
      <mesh position={[rDFf?.x, rDFf?.y, 0.011]}>
        <mesh geometry={ArrowGeometry(min_align_radius)} rotation={[0, 0, robot_rotation.z + angle_to_final_heading]}>
          <meshBasicMaterial color="rgb(255, 0, 0)" opacity={0.5} transparent={true} />
        </mesh>
      </mesh>
    </object3D>
  );
};
