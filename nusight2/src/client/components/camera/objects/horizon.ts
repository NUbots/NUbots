import { Vector4 } from "../../../../shared/math/vector4";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";
import { CameraParams } from "../camera_params";

import { LineProjection } from "./line_projection";

export class HorizonViewModel {
  private readonly params: CameraParams;
  private readonly lineWidth: number;
  private readonly lineProjection: LineProjection;

  constructor(params: CameraParams, lineWidth: number, lineProjection: LineProjection) {
    this.params = params;
    this.lineWidth = lineWidth;
    this.lineProjection = lineProjection;
  }

  static of(
    canvas: Canvas,
    params: CameraParams,
    imageAspectRatio: number,
    lineWidth: number = 10.0,
  ): HorizonViewModel {
    return new HorizonViewModel(params, lineWidth, LineProjection.of(canvas, params.lens, imageAspectRatio));
  }

  readonly horizon = group(() => ({
    children: [
      this.lineProjection.plane({
        axis: this.params.Hcw.z.vec3(),
        color: new Vector4(0, 0, 1, 0.7),
        lineWidth: this.lineWidth,
      }),
    ],
  }));
}
