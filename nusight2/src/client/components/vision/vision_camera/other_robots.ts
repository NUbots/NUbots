import { createTransformer } from "mobx-utils";
import * as THREE from "three";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";
import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { LineProjection } from "../../camera/objects/line_projection";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";

export interface OtherRobotsModel {
  readonly timestamp: number;
  readonly Hcw: Matrix4;
  readonly rRCc: Vector3;
}

export class OtherRobotsViewModel {
  private readonly model: OtherRobotsModel[];
  private readonly params: CameraParams;
  private readonly lineProjection: LineProjection;

  constructor(model: OtherRobotsModel[], params: CameraParams, lineProjection: LineProjection) {
    this.model = model;
    this.params = params;
    this.lineProjection = lineProjection;
  }

  static of(model: OtherRobotsModel[], params: CameraParams, canvas: Canvas, imageAspectRatio: number): OtherRobotsViewModel {
    return new OtherRobotsViewModel(model, params, LineProjection.of(canvas, params.lens, imageAspectRatio));
  }

  readonly robots = group(() => ({
    children: this.model.map((robot) => this.robot(robot)),
  }));

  private robot = createTransformer((m: OtherRobotsModel) => {
    const Hwc = new THREE.Matrix4().copy(m.Hcw.toThree()).invert();
    const Hcc = Matrix4.fromThree(this.params.Hcw.toThree().multiply(Hwc));

    // Transform the axis so that it is in the perspective of the latest camera image.
    const axis = Vector3.fromThree(m.rRCc.toThree().applyMatrix4(Hcc.toThree())).normalize();
    const magenta = new Vector4(1, 0.5, 0, 1);

    return this.lineProjection.cone({
      axis,
      radius: 0.999,
      color: magenta,
      lineWidth: 10,
    });
  });
}
