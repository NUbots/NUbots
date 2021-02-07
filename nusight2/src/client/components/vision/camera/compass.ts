import { computed } from 'mobx'

import { Vector4 } from '../../../math/vector4'
import { group } from '../../three/builders'
import { Canvas } from '../../three/three'
import { CameraParams } from '../camera/model'

import { LineProjection } from './line_projection'

export class CompassViewModel {
  constructor(
    private readonly params: CameraParams,
    private readonly lineProjection: LineProjection,
  ) {}

  static of(canvas: Canvas, params: CameraParams): CompassViewModel {
    return new CompassViewModel(params, LineProjection.of(canvas, params.lens))
  }

  readonly compass = group(() => ({
    children: [this.xPositiveAxis, this.xNegativeAxis, this.yPositiveAxis, this.yNegativeAxis],
  }))

  @computed
  private get xPositiveAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.x.vec3(),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(1, 0, 0, 0.5), // Red
      lineWidth: 5,
    })
  }

  @computed
  private get xNegativeAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.x.vec3().multiplyScalar(-1),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(0, 1, 1, 0.5), // Cyan
      lineWidth: 5,
    })
  }

  @computed
  private get yPositiveAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.y.vec3(),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(0, 1, 0, 0.5), // Green
      lineWidth: 5,
    })
  }

  @computed
  private get yNegativeAxis() {
    return this.lineProjection.planeSegment({
      start: this.params.Hcw.y.vec3().multiplyScalar(-1),
      end: this.params.Hcw.z.vec3().multiplyScalar(-1),
      color: new Vector4(1, 0, 1, 0.5), // Magenta
      lineWidth: 5,
    })
  }
}
