import React from "react";
import { action } from "mobx";
import { computed } from "mobx";

import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";
import { PerspectiveCamera, ThreeFiber } from "../../three/three_fiber";

import { OdometryVisualizerModel } from "./model";
import styles from "./style.module.css";
import * as THREE from "three";
import { observer } from "mobx-react";

@observer
export class OdometryVisualizer extends React.Component<{ model: OdometryVisualizerModel }> {
  private dragger?: Dragger;

  render() {
    return (
      <div className={styles.visualizer}>
        <ThreeFiber
          onWheel={this.onWheel}
          onMouseDown={this.onMouseDown}
          onMouseMove={this.onMouseMove}
          onMouseUp={this.onMouseUp}
        >
          <PerspectiveCamera
            fov={75}
            aspect={1}
            near={0.001}
            far={100}
            up={[0, 0, 1]}
            position={this.cameraPosition.toArray()}
            lookAt={this.rTWw}
          />
          {this.torso}
          {this.floor}
          {this.worldFrame}
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
  }

  private get torso() {
    return (
      <object3D
        position={this.rTWw.toArray()}
        rotation={new THREE.Euler().setFromRotationMatrix(this.model.Hwt.toThree())}
      >
        {this.basis}
        {this.accelerometer}
      </object3D>
    );
  }

  private get basis() {
    return (
      <object3D>
        <arrowHelper args={[new THREE.Vector3(1, 0, 0), undefined, 1, 0xff0000]} />
        <arrowHelper args={[new THREE.Vector3(0, 1, 0), undefined, 1, 0x00ff00]} />
        <arrowHelper args={[new THREE.Vector3(0, 0, 1), undefined, 1, 0x0000ff]} />
      </object3D>
    );
  }

  private get worldFrame() {
    return (
      <object3D position={this.rTWw.toArray()}>
        <arrowHelper args={[new THREE.Vector3(1, 0, 0), undefined, 0.2, 0xff0000]} />
        <arrowHelper args={[new THREE.Vector3(0, 1, 0), undefined, 0.2, 0x00ff00]} />
        <arrowHelper args={[new THREE.Vector3(0, 0, 1), undefined, 0.2, 0x0000ff]} />
      </object3D>
    );
  }

  private get accelerometer() {
    return (
      <arrowHelper
        args={[
          this.model.accelerometer.normalize().toThree(),
          undefined,
          this.model.accelerometer.length / 9.8,
          0xffffff,
        ]}
      />
    );
  }

  private get floor() {
    return <gridHelper args={[100, 100]} rotation={[Math.PI / 2, 0, 0]} />;
  }

  private get model(): OdometryVisualizerModel {
    return this.props.model;
  }

  @computed
  private get cameraPosition(): Vector3 {
    const { distance, pitch, yaw } = this.model.camera;
    const p = pitch - Math.PI / 2;
    const y = -yaw + Math.PI;
    const orbitPosition = new Vector3(Math.sin(p) * Math.cos(y), Math.sin(p) * Math.sin(y), Math.cos(p)).multiplyScalar(
      -distance,
    ); // rCTw
    return orbitPosition.add(this.rTWw);
  }

  @computed
  private get rTWw() {
    return this.model.Hwt.t.vec3();
  }

  @action
  private onWheel = (event: React.WheelEvent) => {
    const { camera } = this.props.model;
    camera.distance = clamp(camera.distance + event.deltaY / 200, 0.01, 10, 0);
  };

  @action
  private onMouseDown = (event: React.MouseEvent) => {
    const {
      model,
      model: {
        camera: { pitch, yaw },
      },
    } = this.props;
    this.dragger = new Dragger(model, pitch, yaw, Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY));
  };

  @action
  private onMouseMove = (event: React.MouseEvent) => {
    if (!this.dragger) {
      return;
    }
    this.dragger.to = Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY);
  };

  @action
  private onMouseUp = (event: React.MouseEvent) => {
    if (!this.dragger) {
      return;
    }
    this.dragger.to = Vector2.of(event.nativeEvent.layerX, event.nativeEvent.layerY);
    this.dragger = undefined;
  };
}

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
