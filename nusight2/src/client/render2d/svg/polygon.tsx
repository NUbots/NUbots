import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { PolygonGeometry } from '../geometry/polygon_geometry'
import { Shape } from '../object/shape'

import { toSvgProps } from './rendering'

type Props = { model: Shape<PolygonGeometry>; world: Transform }
export const Polygon = observer(
  ({
    model: {
      geometry: { points },
      appearance,
    },
  }: Props) => (
    <polygon points={points.map(p => `${p.x},${p.y}`).join(' ')} {...toSvgProps(appearance)} />
  ),
)
