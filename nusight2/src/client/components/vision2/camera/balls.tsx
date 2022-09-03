import { observer } from 'mobx-react'
import React from 'react'
import * as THREE from 'three'

import { Matrix4 } from '../../../math/matrix4'
import { Vector3 } from '../../../math/vector3'

import { ConeView } from './line_projection'
import { Cone } from './model'
import { CameraParams } from './model'
import { Ball } from './model'

export const BallsView = observer(({ balls, params }: { balls: Ball[]; params: CameraParams }) => (
  <object3D>
    {balls.map((ball, i) => (
      <BallView key={i} ball={ball} params={params} />
    ))}
  </object3D>
))

export const BallView = observer(({ ball, params }: { ball: Ball; params: CameraParams }) => {
  const Hwc = new THREE.Matrix4().getInverse(ball.Hcw.toThree())
  const Hcc = Matrix4.fromThree(params.Hcw.toThree().multiply(Hwc))
  // Transform the cone so that it is in the perspective of the latest camera image.
  const { axis, radius } = transform(ball.cone, ball.distance, Hcc)
  return (
    <ConeView axis={axis} radius={radius} color={ball.colour} lineWidth={10} lens={params.lens} />
  )
})

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
