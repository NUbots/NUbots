import React from "react";
import { action } from "mobx";
import { computed } from "mobx";

import { Vector2 } from "../../../../shared/math/vector2";
import { Canvas } from "../../three/three";
import { Three } from "../../three/three";

import { OdometryVisualizerModel } from "./model";
import styles from "./style.module.css";
import { OdometryVisualizerViewModel } from "./view_model";

export class OdometryVisualizer extends React.Component<{ model: OdometryVisualizerModel }> {
  private dragger?: Dragger;

  render() {
    return (
      <div className={styles.visualizer}>
        <Three
          stage={this.stage}
          onWheel={this.onWheel}
          onMouseDown={this.onMouseDown}
          onMouseMove={this.onMouseMove}
          onMouseUp={this.onMouseUp}
        />
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

  private readonly stage = (canvas: Canvas) => {
    const cameraViewModel = OdometryVisualizerViewModel.of(canvas, this.props.model);
    return computed(() => [cameraViewModel.stage]);
  };

  @action.bound
  private onWheel(deltaY: number) {
    const { camera } = this.props.model;
    camera.distance = clamp(camera.distance + deltaY / 200, 0.01, 10, 0);
  }

  @action.bound
  private onMouseDown(x: number, y: number) {
    const {
      model,
      model: {
        camera: { pitch, yaw },
      },
    } = this.props;
    this.dragger = new Dragger(model, pitch, yaw, Vector2.of(x, y));
  }

  @action.bound
  private onMouseMove(x: number, y: number) {
    if (!this.dragger) {
      return;
    }
    this.dragger.to = Vector2.of(x, y);
  }

  @action.bound
  private onMouseUp(x: number, y: number) {
    if (!this.dragger) {
      return;
    }
    this.dragger.to = Vector2.of(x, y);
    this.dragger = undefined;
  }
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
