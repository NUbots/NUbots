import { createTransformer } from 'mobx-utils'
import * as THREE from 'three'

import { UnreachableError } from '../../../../shared/base/unreachable_error'
import { FieldDimensions } from '../../../../shared/field/dimensions'
import { Vector3 } from '../../../math/vector3'
import { Vector4 } from '../../../math/vector4'
import { group } from '../../three/builders'
import { Canvas } from '../../three/three'
import { CameraParams } from '../camera/model'
import { Goal } from '../camera/model'

import { LineProjection } from './line_projection'

export class GoalsViewModel {
  private readonly model: Goal[]
  private readonly params: CameraParams
  private readonly lineProjection: LineProjection

  constructor(model: Goal[], params: CameraParams, lineProjection: LineProjection) {
    this.model = model
    this.params = params
    this.lineProjection = lineProjection
  }

  static of(model: Goal[], canvas: Canvas, params: CameraParams): GoalsViewModel {
    return new GoalsViewModel(model, params, LineProjection.of(canvas, params.lens))
  }

  readonly goals = group(() => ({
    children: this.model.map(goal => this.goal(goal)),
  }))

  private goal = createTransformer((goal: Goal) => {
    // Transform the goal line so that it is in the perspective of the latest camera image.
    const Hwc = new THREE.Matrix4().getInverse(goal.Hcw.toThree())
    const Hcc = this.params.Hcw.toThree().multiply(Hwc)
    const bottom = goal.post.bottom.toThree().multiplyScalar(goal.post.distance)
    return this.lineProjection.planeSegment({
      start: Vector3.fromThree(
        bottom
          .clone()
          .applyMatrix4(Hwc)
          .add(new THREE.Vector3(0, 0, FieldDimensions.postYear2017().goalCrossbarHeight))
          .applyMatrix4(this.params.Hcw.toThree())
          .normalize(),
      ),
      end: Vector3.fromThree(bottom.clone().applyMatrix4(Hcc).normalize()),
      color: getColor(goal.side),
      lineWidth: 10,
    })
  })
}

function getColor(side: Goal['side']) {
  switch (side) {
    case 'left':
      return new Vector4(1.0, 1.0, 0, 1.0) // Yellow
    case 'right':
      return new Vector4(0.0, 1.0, 1.0, 1.0) // Cyan
    case 'unknown':
      return new Vector4(1.0, 0.0, 1.0, 1.0) // Magenta
    default:
      throw new UnreachableError(side)
  }
}
