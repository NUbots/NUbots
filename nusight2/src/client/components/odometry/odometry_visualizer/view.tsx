import React, { useState } from "react";
import { Object3DProps } from "@react-three/fiber";
import { observer } from "mobx-react";
import * as THREE from "three";

import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";
import { PerspectiveCamera, ThreeFiber } from "../../three/three_fiber";

import { OdometryVisualizerModel } from "./model";
import styles from "./style.module.css";

export const OdometryVisualizer = observer(({ model }: { model: OdometryVisualizerModel }) => {
  const [dragger, setDragger] = useState<Dragger | undefined>(undefined);
  const rTWw = model.Hwt.t.vec3();

  const cameraPosition = React.useMemo(() => {
    const { distance, pitch, yaw } = model.camera;
    const p = pitch - Math.PI / 2;
    const y = -yaw + Math.PI;
    const orbitPosition = new Vector3(Math.sin(p) * Math.cos(y), Math.sin(p) * Math.sin(y), Math.cos(p)).multiplyScalar(
      -distance,
    ); // rCTw
    return orbitPosition.add(rTWw);
  }, [model.camera.distance, model.camera.yaw, model.camera.pitch]);

  const onWheel = (event: React.WheelEvent) => {
    const { camera } = model;
    camera.distance = clamp(camera.distance + event.deltaY / 200, 0.01, 10, 0);
  };

  const onMouseDown = (event: React.MouseEvent) => {
    const {
      camera: { pitch, yaw },
    } = model;
    setDragger(new Dragger(model, pitch, yaw, Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY)));
  };

  const onMouseMove = (event: React.MouseEvent) => {
    if (!dragger) {
      return;
    }
    dragger.to = Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY);
  };

  const onMouseUp = (event: React.MouseEvent) => {
    if (!dragger) {
      return;
    }
    dragger.to = Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY);
    setDragger(undefined);
  };

  return (
    <div className={styles.visualizer}>
      <ThreeFiber onWheel={onWheel} onMouseDown={onMouseDown} onMouseMove={onMouseMove} onMouseUp={onMouseUp}>
        <PerspectiveCamera
          fov={75}
          aspect={1}
          near={0.001}
          far={100}
          up={[0, 0, 1]}
          position={cameraPosition.toArray()}
          lookAt={rTWw}
        />
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

class Dragger {
  private _to: Vector2;

  constructor(
    private readonly model: OdometryVisualizerModel,
    private readonly fromPitch: number,
    private readonly fromYaw: number,
    private readonly from: Vector2,
  ) {
    this._to = from;
  }

  set to(to: Vector2) {
    this._to = to;
    this.update();
  }

  private update() {
    const delta = this._to.subtract(this.from);
    const scale = 1 / 100;
    this.model.camera.pitch = clamp(this.fromPitch - delta.y / 100, -Math.PI / 2, Math.PI / 2);
    this.model.camera.yaw = this.fromYaw + scale * delta.x;
  }
}

const clamp = (x: number, min: number, max: number, eps = 1e-9): number => {
  return Math.max(min + eps, Math.min(max - eps, x));
};
