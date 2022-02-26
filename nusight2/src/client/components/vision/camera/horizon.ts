import { Vector4 } from '../../../math/vector4'
import { group } from '../../three/builders'
import { Canvas } from '../../three/three'

import { LineProjection } from './line_projection'
import { CameraParams } from './model'

export class HorizonViewModel {
  private readonly params: CameraParams
  private readonly lineProjection: LineProjection

  constructor(params: CameraParams, lineProjection: LineProjection) {
    this.params = params
    this.lineProjection = lineProjection
  }

  static of(canvas: Canvas, params: CameraParams): HorizonViewModel {
    return new HorizonViewModel(params, LineProjection.of(canvas, params.lens))
  }

  readonly horizon = group(() => ({
    children: [
      this.lineProjection.plane({
        axis: this.params.Hcw.z.vec3(),
        color: new Vector4(0, 0, 1, 0.7),
        lineWidth: 10,
      }),
    ],
  }))
}
