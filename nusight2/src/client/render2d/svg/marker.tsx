import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { MarkerGeometry } from '../geometry/marker_geometry'
import { Shape } from '../object/shape'

import { toSvgProps } from './rendering'

type Props = { model: Shape<MarkerGeometry>; world: Transform }
export const Marker = observer(
  ({
    model: {
      geometry: { x, y, radius, heading },
      appearance,
    },
  }: Props) => {
    const rotation = 135.0 + (180.0 / Math.PI) * Math.atan2(heading.y, heading.x)
    return (
      <path
        d="M-1 -1L0 -1A1 1 270 1 1 -1 0Z"
        transform={`scale(${radius}) rotate(${rotation}) translate(${x}, ${y})`}
        {...toSvgProps(appearance)}
      />
    )
  },
)
