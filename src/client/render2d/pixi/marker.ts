import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'

import { MarkerGeometry } from '../geometry/marker_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderMarker = createTransformer((shape: Shape<MarkerGeometry>): Graphics => {

  const { x, y, radius, heading } = shape.geometry

  const headingAngle = Math.atan2(heading.y, heading.x)
  const arcDistance = 3 * Math.PI * 0.5

  // By default, the arc startAngle begins on the positive x-axis and rotates clockwise. If the startAngle and
  // endAngle are offset by a quadrant, the arc will point toward the positive x-axis instead of starting there.
  const startAngleOffset = Math.PI * 0.25
  const startAngle = headingAngle + startAngleOffset
  const endAngle = headingAngle + arcDistance + startAngleOffset
  // The diagonal length of a unit square.
  const sqrt2 = Math.sqrt(2)

  const g = new Graphics()
  applyAppearance(g, shape.appearance, g => {
    g.arc(x, y, radius, startAngle, endAngle)
    g.lineTo(
      x + sqrt2 * radius * heading.x,
      y + sqrt2 * radius * heading.y,
    )
    g.closePath()
  })

  return g
})
