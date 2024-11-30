import React from "react";
import { OrbitControls } from "@react-three/drei";
import { Object3DProps } from "@react-three/fiber";
import { observer } from "mobx-react";
import * as THREE from "three";

import { Vector3 } from "../../../../shared/math/vector3";
import { PerspectiveCamera, ThreeFiber } from "../../three/three_fiber";

import { OdometryVisualizerModel } from "./model";
import styles from "./style.module.css";

export const OdometryVisualizer = observer(({ model }: { model: OdometryVisualizerModel }) => {
  const rTWw = model.Hwt.t.vec3();
  return (
    <div className={styles.visualizer}>
      <ThreeFiber>
        <PerspectiveCamera fov={75} aspect={1} near={0.001} far={100} up={[0, 0, 1]} />
        <OrbitControls enableDamping={true} enablePan={false} target={rTWw.toArray()} />
        <Torso
          accelerometer={model.accelerometer}
          position={rTWw.toArray()}
          rotation={new THREE.Euler().setFromRotationMatrix(model.Hwt.toThree())}
        />
        <Floor />
        <WorldFrame position={rTWw.toArray()} />
      </ThreeFiber>
      <div className={styles.legend}>
        <div className={styles.item}>
          <div className={styles.color} style={{ backgroundColor: "red" }} />
          <div className={styles.color} style={{ backgroundColor: "green" }} />
          <div className={styles.color} style={{ backgroundColor: "blue" }} />
          <span>Hwt</span>
        </div>
        <div className={styles.item}>
          <div className={styles.color} style={{ backgroundColor: "white" }} />
          <span>Accelerometer</span>
        </div>
      </div>
    </div>
  );
});

const Torso = (props: { accelerometer: Vector3 } & Object3DProps) => (
  <object3D {...props}>
    <Basis />
    <Accelerometer accelerometer={props.accelerometer} />
  </object3D>
);

const Basis = () => (
  <object3D>
    <arrowHelper args={[new THREE.Vector3(1, 0, 0), undefined, 1, 0xff0000]} />
    <arrowHelper args={[new THREE.Vector3(0, 1, 0), undefined, 1, 0x00ff00]} />
    <arrowHelper args={[new THREE.Vector3(0, 0, 1), undefined, 1, 0x0000ff]} />
  </object3D>
);

const WorldFrame = (props: Object3DProps) => (
  <object3D {...props}>
    <arrowHelper args={[new THREE.Vector3(1, 0, 0), undefined, 0.2, 0xff0000]} />
    <arrowHelper args={[new THREE.Vector3(0, 1, 0), undefined, 0.2, 0x00ff00]} />
    <arrowHelper args={[new THREE.Vector3(0, 0, 1), undefined, 0.2, 0x0000ff]} />
  </object3D>
);

const Accelerometer = ({ accelerometer }: { accelerometer: Vector3 }) => (
  <arrowHelper args={[accelerometer.normalize().toThree(), undefined, accelerometer.length / 9.8, 0xffffff]} />
);

const Floor = () => <gridHelper args={[100, 100]} rotation={[Math.PI / 2, 0, 0]} />;
