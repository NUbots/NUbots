import { observer } from 'mobx-react'
import React from 'react'
import { Vector4 } from '../../../math/vector4'
import { PlaneView } from './line_projection'

import { CameraParams } from './model'

export const HorizonView = observer(({ params }: { params: CameraParams }) => (
  <PlaneView
    axis={params.Hcw.z.vec3()}
    color={new Vector4(0, 0, 1, 0.7)}
    lineWidth={10}
    lens={params.lens}
  />
))
