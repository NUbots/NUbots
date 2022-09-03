import { observer } from 'mobx-react'
import React from 'react'
import * as THREE from 'three'

import { UnreachableError } from '../../../../shared/base/unreachable_error'
import { FieldDimensions } from '../../../../shared/field/dimensions'
import { Vector3 } from '../../../math/vector3'
import { Vector4 } from '../../../math/vector4'
import { CameraParams } from '../camera/model'
import { Goal } from '../camera/model'
import { PlaneSegmentView } from './line_projection'

export const GoalsView = observer(({ goals, params }: { goals: Goal[]; params: CameraParams }) => (
  <object3D>
    {goals.map((goal, i) => (
      <GoalView key={i} goal={goal} params={params} />
    ))}
  </object3D>
))

export const GoalView = observer(({ goal, params }: { goal: Goal; params: CameraParams }) => {
  // Transform the goal line so that it is in the perspective of the latest camera image.
  const Hwc = new THREE.Matrix4().getInverse(goal.Hcw.toThree())
  const Hcc = params.Hcw.toThree().multiply(Hwc)
  const bottom = goal.post.bottom.toThree().multiplyScalar(goal.post.distance)
  return (
    <PlaneSegmentView
      start={Vector3.fromThree(
        bottom
          .clone()
          .applyMatrix4(Hwc)
          .add(new THREE.Vector3(0, 0, FieldDimensions.postYear2017().goalCrossbarHeight))
          .applyMatrix4(params.Hcw.toThree())
          .normalize(),
      )}
      end={Vector3.fromThree(bottom.clone().applyMatrix4(Hcc).normalize())}
      color={getColor(goal.side)}
      lineWidth={10}
      lens={params.lens}
    />
  )
})

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
