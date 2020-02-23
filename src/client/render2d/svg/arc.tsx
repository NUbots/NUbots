import { observer } from 'mobx-react'
import React from 'react'

import { Transform } from '../../math/transform'
import { Vector2 } from '../../math/vector2'
import { ArcGeometry } from '../geometry/arc_geometry'
import { Shape } from '../object/shape'

import { toSvgProps } from './rendering'

type Props = { model: Shape<ArcGeometry>; world: Transform }
export const Arc = observer(({ model: { geometry, appearance } }: Props) => {
  const { origin, radius, startAngle, endAngle, anticlockwise } = geometry

  // Is this arc empty?
  if (radius < 0) {
    throw new Error(`Negative radius: ${radius}`)
  }

  // Calculate out our start and end points on the circle
  const p0 = Vector2.fromPolar(radius, startAngle).add(origin)
  const p1 = Vector2.fromPolar(radius, endAngle).add(origin)

  // Our cw value must be a 0/1 not true/false
  const cw = +!anticlockwise
  const da = anticlockwise ? startAngle - endAngle : endAngle - startAngle

  // Move to (x0,y0).
  let path = `M${p0.x} ${p0.y}`

  // Draw the arc
  path += `A${radius} ${radius} 0 ${+(da >= Math.PI)} ${cw} ${p1.x} ${p1.y}`

  return <path d={path} {...toSvgProps(appearance)} />
})
