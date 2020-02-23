import { observable } from 'mobx'

import { Matrix4 } from '../../../math/matrix4'
import { Vector2 } from '../../../math/vector2'
import { Vector3 } from '../../../math/vector3'
import { Vector4 } from '../../../math/vector4'
import { Image } from '../image'

type DrawOptions = {
  drawImage: boolean,
  drawCompass: boolean,
  drawHorizon: boolean,
  drawGreenhorizon: boolean,
  drawBalls: boolean,
  drawGoals: boolean
}

export class CameraModel {
  @observable.ref id: number
  @observable.ref name: string
  @observable.ref image: Image
  @observable.ref params: CameraParams

  @observable.ref greenhorizon?: GreenHorizon
  @observable.ref balls?: Ball[]
  @observable.ref goals?: Goal[]

  @observable.shallow drawOptions: DrawOptions

  constructor({ id, name, image, params, greenhorizon, balls, goals, drawOptions }: {
    id: number,
    name: string,
    image: Image
    params: CameraParams
    greenhorizon?: GreenHorizon
    balls?: Ball[]
    goals?: Goal[]
    drawOptions: DrawOptions
  }) {
    this.id = id
    this.name = name
    this.image = image
    this.params = params
    this.greenhorizon = greenhorizon
    this.balls = balls
    this.goals = goals
    this.drawOptions = drawOptions
  }

  static of(opts: {
    id: number,
    name: string,
    image: Image,
    params: CameraParams,
    greenhorizon?: GreenHorizon,
    balls?: Ball[],
    goals?: Goal[]
  }) {
    return new CameraModel({
      drawOptions: {
        drawImage: true,
        drawCompass: true,
        drawHorizon: true,
        drawGreenhorizon: true,
        drawBalls: true,
        drawGoals: true,
      },
      ...opts,
    })
  }
}

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

export interface Ball {
  readonly timestamp: number
  readonly Hcw: Matrix4
  readonly cone: Cone
  readonly distance: number
  readonly colour: Vector4
}

export interface Goal {
  readonly timestamp: number
  readonly Hcw: Matrix4
  readonly side: 'left' | 'right' | 'unknown'
  readonly post: {
    readonly top: Vector3
    readonly bottom: Vector3
    readonly distance: number
  }
}

export interface Cone {
  readonly axis: Vector3
  readonly radius: number
}
