import { computed } from "mobx";
import * as THREE from "three";

import { range } from "../../../../shared/base/range";
import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";

import { LineProjection } from "./line_projection";

export class DistanceViewModel {
  private readonly params: CameraParams;
  private readonly lineProjection: LineProjection;
  private readonly majorStep: number;
  private readonly minorLines: number;
  private readonly maxDistance: number;

  constructor({
    params,
    lineProjection,
    majorStep,
    minorLines,
    maxDistance,
  }: {
    params: CameraParams;
    lineProjection: LineProjection;
    majorStep: number;
    minorLines: number;
    maxDistance: number;
  }) {
    this.params = params;
    this.lineProjection = lineProjection;
    this.majorStep = majorStep;
    this.minorLines = minorLines;
    this.maxDistance = maxDistance;
  }

  static of(canvas: Canvas, params: CameraParams, imageAspectRatio: number): DistanceViewModel {
    return new DistanceViewModel({
      params,
      lineProjection: LineProjection.of(canvas, params.lens, imageAspectRatio),
      majorStep: 1,
      minorLines: 3,
      maxDistance: 50,
    });
  }

  readonly distance = group(() => ({
    children: range(((this.minorLines + 1) * this.maxDistance) / this.majorStep).map((i) =>
      this.lineProjection.cone({
        axis: this.params.Hcw.z.vec3().multiplyScalar(-1),
        radius: Math.cos(Math.atan((i + 1) / (this.minorLines + 1) / this.cameraHeight)),
        color: new Vector4(1, 1, 1, (i + 1) % (this.minorLines + 1) ? 0.2 : 0.4),
        lineWidth: (i + 1) % (this.minorLines + 1) ? 2 : 3,
      }),
    ),
  }));

  @computed
  private get cameraHeight(): number {
    return new THREE.Matrix4().copy(this.params.Hcw.toThree()).invert().elements[15];
  }
}
