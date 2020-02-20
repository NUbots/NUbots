import { observable } from 'mobx'

import { Matrix4 } from '../../../math/matrix4'
import { Vector2 } from '../../../math/vector2'
import { Vector3 } from '../../../math/vector3'

export class CameraParams {
  @observable.ref Hcw: Matrix4
  @observable.ref lens: Lens

  constructor({ Hcw, lens }: { Hcw: Matrix4, lens: Lens }) {
    this.Hcw = Hcw
    this.lens = lens
  }
}

export class Lens {
  @observable.ref projection: Projection
  @observable.ref focalLength: number
  @observable.ref centre?: Vector2
  @observable.ref distortionCoeffecients?: Vector2

  constructor({ projection, focalLength, centre, distortionCoeffecients }: {
    projection: Projection,
    focalLength: number,
    centre?: Vector2,
    distortionCoeffecients?: Vector2
  }) {
    this.projection = projection
    this.focalLength = focalLength
    this.centre = centre
    this.distortionCoeffecients = distortionCoeffecients
  }
}

export enum Projection {
  UNKNOWN = 0,
  RECTILINEAR = 1,
  EQUIDISTANT = 2,
  EQUISOLID = 3,
}

export class GreenHorizon {
  /** A list of world space camera unit-vector rays. */
  @observable.ref horizon: Vector3[]

  /** The world to camera transform, at the time the green horizon was measured. */
  @observable.ref Hcw: Matrix4

  constructor({ horizon, Hcw }: { horizon?: Vector3[], Hcw?: Matrix4 }) {
    this.horizon = horizon ?? []
    this.Hcw = Hcw ?? Matrix4.of()
  }
}
