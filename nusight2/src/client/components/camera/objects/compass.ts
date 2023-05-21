// Compass and horizon and distance circles are ENU

import { computed } from "mobx";

import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";

import { LineProjection } from "./line_projection";

export class CompassViewModel {
  constructor(
    private readonly params: CameraParams,
    private readonly lineWidth: number,
    private readonly lineProjection: LineProjection,
  ) {}

  static of(canvas: Canvas, params: CameraParams, imageAspectRatio: number, lineWidth: number = 5.0): CompassViewModel {
    return new CompassViewModel(params, lineWidth, LineProjection.of(canvas, params.lens, imageAspectRatio));
  }

  readonly compass = group(() => ({
    children: [this.xPositiveAxis, this.xNegativeAxis, this.yPositiveAxis, this.yNegativeAxis],
  }));

  @computed
  private get xPositiveAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.x.vec3(),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(1, 0, 0, 0.5), // Red
      lineWidth: this.lineWidth,
    });
  }

  @computed
  private get xNegativeAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.x.vec3().multiplyScalar(-1),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(0, 1, 1, 0.5), // Cyan
      lineWidth: this.lineWidth,
    });
  }

  @computed
  private get yPositiveAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.y.vec3(),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(0, 1, 0, 0.5), // Green
      lineWidth: this.lineWidth,
    });
  }

  @computed
  private get yNegativeAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.y.vec3().multiplyScalar(-1),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(1, 0, 1, 0.5), // Magenta
      lineWidth: this.lineWidth,
    });
  }
}
