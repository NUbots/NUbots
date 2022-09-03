import { observer } from 'mobx-react'
import React from 'react'

import { Vector4 } from '../../../math/vector4'
import { CameraParams } from '../camera/model'
import { PlaneSegmentView } from './line_projection'

export const CompassView = observer(({ params }: { params: CameraParams }) => (
  <object3D>
    <PlaneSegmentView
      start={params.Hcw.x.vec3()}
      end={params.Hcw.z.vec3().multiplyScalar(-1)}
      color={new Vector4(1, 0, 0, 0.5)}
      lineWidth={5}
      lens={params.lens}
    />
    <PlaneSegmentView
      start={params.Hcw.x.vec3().multiplyScalar(-1)}
      end={params.Hcw.z.vec3().multiplyScalar(-1)}
      color={new Vector4(0, 1, 1, 0.5)}
      lineWidth={5}
      lens={params.lens}
    />
    <PlaneSegmentView
      start={params.Hcw.y.vec3()}
      end={params.Hcw.z.vec3().multiplyScalar(-1)}
      color={new Vector4(0, 1, 0, 0.5)}
      lineWidth={5}
      lens={params.lens}
    />
    <PlaneSegmentView
      start={params.Hcw.y.vec3().multiplyScalar(-1)}
      end={params.Hcw.z.vec3().multiplyScalar(-1)}
      color={new Vector4(1, 0, 1, 0.5)}
      lineWidth={5}
      lens={params.lens}
    />
  </object3D>
))
