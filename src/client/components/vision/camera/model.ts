import { observable } from 'mobx'

import { Image } from '../../../image_decoder/image_decoder'
import { Matrix4 } from '../../../math/matrix4'
import { Vector2 } from '../../../math/vector2'
import { Vector3 } from '../../../math/vector3'
import { VisionRobotModel } from '../model'

export interface VisualMesh {
  readonly neighbours: number[]
  readonly coordinates: number[]
  readonly classifications: { dim: number, values: number[] }
}

export interface VisionImage extends Image {
  readonly Hcw: Matrix4
  readonly lens: {
    readonly projection: number
    readonly focalLength: number
    readonly centre: Vector2
  }
}

export interface Ball {
  readonly timestamp: number
  readonly Hcw: Matrix4
  readonly cone: {
    readonly axis: Vector3
    readonly gradient: number
  }
}

export interface Goal {
  readonly timestamp: number
  readonly Hcw: Matrix4
  readonly frustum: {
    readonly tl: Vector3
    readonly tr: Vector3
    readonly bl: Vector3
    readonly br: Vector3
  }
}

export class CameraModel {
  readonly id: number

  @observable.ref visualmesh?: VisualMesh
  @observable.ref image?: VisionImage
  @observable.ref balls: Ball[]
  @observable.ref goals: Goal[]
  @observable.ref name: string

  constructor(private model: VisionRobotModel, { id, name, balls, goals }: {
    id: number
    name: string
    balls: Ball[]
    goals: Goal[]
  }) {
    this.id = id
    this.name = name
    this.balls = balls
    this.goals = goals
  }

  static of(model: VisionRobotModel, { id, name }: { id: number, name: string }) {
    return new CameraModel(model, { id, name, balls: [], goals: [] })
  }
}
