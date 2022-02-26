import { createTransformer } from 'mobx-utils'
import * as THREE from 'three'

import { Matrix4 } from '../../../math/matrix4'
import { Vector3 } from '../../../math/vector3'
import { group } from '../../three/builders'
import { Canvas } from '../../three/three'

import { LineProjection } from './line_projection'
import { CameraParams } from './model'
import { Cone } from './model'
import { Ball } from './model'

export class BallsViewModel {
  private readonly model: Ball[]
  private readonly params: CameraParams
  private readonly lineProjection: LineProjection

  constructor(model: Ball[], params: CameraParams, lineProjection: LineProjection) {
    this.model = model
    this.params = params
    this.lineProjection = lineProjection
  }

  static of(model: Ball[], canvas: Canvas, params: CameraParams): BallsViewModel {
    return new BallsViewModel(model, params, LineProjection.of(canvas, params.lens))
  }

  readonly balls = group(() => ({
    children: this.model.map(ball => this.ball(ball)),
  }))

  private ball = createTransformer((m: Ball) => {
    const Hwc = new THREE.Matrix4().getInverse(m.Hcw.toThree())
    const Hcc = Matrix4.fromThree(this.params.Hcw.toThree().multiply(Hwc))
    // Transform the cone so that it is in the perspective of the latest camera image.
    const { axis, radius } = transform(m.cone, m.distance, Hcc)
    return this.lineProjection.cone({
      axis,
      radius,
      color: m.colour,
      lineWidth: 10,
    })
  })
}

export function transform(cone: Cone, distance: number, transform: Matrix4): Cone {
  const oldPoint = cone.axis.toThree().multiplyScalar(distance)
  const newPoint = oldPoint.applyMatrix4(transform.toThree())
  const newDistanceSqr = newPoint.dot(newPoint)
  return {
    axis: Vector3.fromThree(newPoint.normalize()),
    // Source: https://en.wikipedia.org/wiki/Angular_diameter#Formula
    // Takes `radius = cos(asin(radiusActual / distance))` and solves for the new radius given a new distance.
    // Solve sin(acos(r_1)) * d_1 = sin(acos(r_2)) * d_2 for r_2
    // Simplifies to:
    radius:
      Math.sqrt(distance ** 2 * cone.radius ** 2 - distance ** 2 + newDistanceSqr) /
      Math.sqrt(newDistanceSqr),
  }
}
