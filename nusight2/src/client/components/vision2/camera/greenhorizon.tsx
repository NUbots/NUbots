import { observer } from 'mobx-react'
import { useMemo } from 'react'
import React from 'react'
import * as THREE from 'three'

import { Matrix4 } from '../../../math/matrix4'
import { Vector3 } from '../../../math/vector3'
import { Vector4 } from '../../../math/vector4'

import { PlaneSegmentView } from './line_projection'
import { GreenHorizon } from './model'
import { CameraParams } from './model'

export const GreenHorizonView = observer(
  ({ greenHorizon, params }: { greenHorizon: GreenHorizon; params: CameraParams }) => {
    // TODO: make computed?
    const rays = useMemo(() => {
      const { horizon, Hcw: greenHorizonHcw } = greenHorizon
      const imageHcw = params.Hcw
      const greenHorizonHwc = Matrix4.fromThree(
        new THREE.Matrix4().getInverse(greenHorizonHcw.toThree()),
      )
      const rCWw = greenHorizonHwc.t.vec3()
      return horizon.map(ray =>
        Vector3.fromThree(
          ray
            .toThree() // rUCw
            // Project world space unit vector onto the world/field ground, giving us a camera to field vector in world space.
            .multiplyScalar(ray.z !== 0 ? -greenHorizonHwc.t.z / ray.z : 1) // rFCw
            // Get the world to field vector, so that we can...
            .add(rCWw.toThree()) // rFWw = rFCw + rCWw
            // ...apply the camera image's world to camera transform, giving us a corrected camera space vector.
            .applyMatrix4(imageHcw.toThree()) // rFCc
            // Normalize to get the final camera space direction vector/ray.
            .normalize(), // rUCc
        ),
      )
    }, [greenHorizon, params])

    return (
      <object3D>
        {greenHorizon.horizon.map((_, index) => {
          return (
            index >= 1 && (
              <PlaneSegmentView
                key={index}
                lens={params.lens}
                start={rays[index - 1]}
                end={rays[index]}
                color={new Vector4(0, 0.8, 0, 0.8)}
                lineWidth={10}
              />
            )
          )
        })}
      </object3D>
    )
  },
)
