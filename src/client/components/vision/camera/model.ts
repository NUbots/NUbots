import { action } from 'mobx'
import { computed } from 'mobx'
import { observable } from 'mobx'

import { Image } from '../../../image_decoder/image_decoder'
import { Matrix4 } from '../../../math/matrix4'
import { Vector2 } from '../../../math/vector2'
import { Vector3 } from '../../../math/vector3'
import { Vector4 } from '../../../math/vector4'
import { VisionRobotModel } from '../model'

export interface GreenHorizon {
  readonly horizon: Vector3[]
  readonly Hcw: Matrix4
}

export interface VisualMesh {
  readonly neighbours: number[]
  readonly rays: number[]
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
    readonly radius: number
  }
  readonly colour: Vector4
}

export interface Goal {
  readonly timestamp: number
  readonly Hcw: Matrix4
  readonly side: 'left' | 'right' | 'unknown'
  readonly post: {
    readonly top: Vector3
    readonly bottom: Vector3
  }
}

export class CameraModel {
  readonly id: number

  @observable.ref greenhorizon?: GreenHorizon
  @observable.ref visualmesh?: VisualMesh
  @observable.ref image?: VisionImage
  @observable.ref balls: Ball[]
  @observable.ref goals: Goal[]
  @observable.ref name: string
  @observable draw = {
    image: true,
    compass: true,
    horizon: true,
    visualmesh: true,
    greenhorizon: true,
    balls: true,
    goals: true,
  }

  @computed
  get drawOptions() {
    return [
      {
        label: 'Image',
        enabled: this.draw.image,
        toggle: action(() => this.draw.image = !this.draw.image),
      },
      {
        label: 'Compass',
        enabled: this.draw.compass,
        toggle: action(() => this.draw.compass = !this.draw.compass),
      },
      {
        label: 'Horizon',
        enabled: this.draw.horizon,
        toggle: action(() => this.draw.horizon = !this.draw.horizon),
      },
      {
        label: 'Visual Mesh',
        enabled: this.draw.visualmesh,
        toggle: action(() => this.draw.visualmesh = !this.draw.visualmesh),
      },
      {
        label: 'Green Horizon',
        enabled: this.draw.greenhorizon,
        toggle: action(() => this.draw.greenhorizon = !this.draw.greenhorizon),
      },
      {
        label: 'Balls',
        enabled: this.draw.balls,
        toggle: action(() => this.draw.balls = !this.draw.balls),
      },
      {
        label: 'Goals',
        enabled: this.draw.goals,
        toggle: action(() => this.draw.goals = !this.draw.goals),
      },
    ]
  }

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
