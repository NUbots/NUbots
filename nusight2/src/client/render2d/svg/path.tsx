import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { PathGeometry } from '../geometry/path_geometry'
import { Shape } from '../object/shape'

import { toSvgProps } from './rendering'

type Props = { model: Shape<PathGeometry>; world: Transform }
export const Path = observer(
  ({
    model: {
      geometry: { points },
      appearance,
    },
  }: Props) => {
    const start = points[0]
    const path = `M${start.x},${start.y}` + points.map(p => `L${p.x},${p.y}`)

    return <path d={path} {...toSvgProps(appearance)} fill="none" />
  },
)
