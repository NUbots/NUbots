import React from "react";
import * as THREE from "three";

import { LocalisationRobotModel } from "../../robot_model";
import { ArrowGeometry } from "../arrow_geometry/ArrowGeometry";

export const WalkPathVisualiser = ({ model }: { model: LocalisationRobotModel }) => {
  if (!model.Hfd || !model.Hfr) {
    return null;
  }
  const rDFf = model.Hfd?.decompose().translation;
  const rTFf = model.Hft.decompose().translation;
  const robot_rotation = new THREE.Euler().setFromQuaternion(model.Hft.decompose().rotation.toThree(), "XYZ");
  const target_rotation = new THREE.Euler().setFromQuaternion(model.Hfd.decompose().rotation.toThree(), "XYZ");
  const min_align_radius = model.min_align_radius;
  const max_align_radius = model.max_align_radius;
  const min_angle_error = model.min_angle_error;
  const max_angle_error = model.max_angle_error;
  const angle_to_final_heading = model.angle_to_final_heading;
  const Rfr = new THREE.Quaternion(
    model.Hfr?.decompose().rotation.x,
    model.Hfr?.decompose().rotation.y,
    model.Hfr?.decompose().rotation.z,
    model.Hfr?.decompose().rotation.w,
  );
  const vRf = model.velocity_target.toThree().applyQuaternion(Rfr);

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
