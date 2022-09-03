import { observer } from 'mobx-react'
import { useMemo } from 'react'
import React from 'react'
import * as THREE from 'three'

import { range } from '../../../../shared/base/range'
import { Vector4 } from '../../../math/vector4'
import { CameraParams } from '../camera/model'

import { ConeView } from './line_projection'

export const DistanceView = observer(
  ({
    params,
    majorStep = 1,
    minorLines = 3,
    maxDistance = 5,
  }: {
    params: CameraParams
    majorStep?: number
    minorLines?: number
    maxDistance?: number
  }) => {
    const cameraHeight = useMemo(
      () => new THREE.Matrix4().getInverse(params.Hcw.toThree()).elements[15],
      [params.Hcw],
    )
    return (
      <object3D>
        {range(((minorLines + 1) * maxDistance) / majorStep).map(i => (
          <ConeView
            key={i}
            axis={params.Hcw.z.vec3().multiplyScalar(-1)}
            radius={Math.cos(Math.atan((i + 1) / (minorLines + 1) / cameraHeight))}
            color={new Vector4(1, 1, 1, (i + 1) % (minorLines + 1) ? 0.2 : 0.4)}
            lineWidth={(i + 1) % (minorLines + 1) ? 2 : 3}
            lens={params.lens}
          />
        ))}
      </object3D>
    )
  },
)
