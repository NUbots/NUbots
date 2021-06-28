import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { ArrowGeometry } from '../geometry/arrow_geometry'
import { Shape } from '../object/shape'

import { toSvgProps } from './rendering'

type Props = { model: Shape<ArrowGeometry>; world: Transform }
export const Arrow = observer(({ model: { geometry, appearance } }: Props) => {
  const { origin, direction, width, length, headWidth, headLength } = geometry
  const w = width * 0.5
  const hl = headLength * 0.5
  const hw = headWidth * 0.5
  const r = (180 / Math.PI) * Math.atan2(direction.y, direction.x)

  let path = `M0 ${-w}`
  path += `L${length - hl} ${-w}`
  path += `L${length - hl} ${-hw}`
  path += `L${length} 0`
  path += `L${length - hl} ${hw}`
  path += `L${length - hl} ${w}`
  path += `L0 ${w}`

  return (
    <path
      d={path}
      transform={`translate(${origin.x},${origin.y}) rotate(${r})`}
      {...toSvgProps(appearance)}
    />
  )
})
