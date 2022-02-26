import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { CircleGeometry } from '../geometry/circle_geometry'
import { Shape } from '../object/shape'

import { toSvgProps } from './rendering'

type Props = { model: Shape<CircleGeometry>; world: Transform }
export const Circle = observer(
  ({
    model: {
      geometry: { x, y, radius },
      appearance,
    },
  }: Props) => <circle cx={x} cy={y} r={radius} {...toSvgProps(appearance)} />,
)
