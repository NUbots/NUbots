import { BasicAppearance } from '../appearance/basic_appearance'
import { MarkerGeometry } from '../geometry/marker_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export function renderMarker(ctx: CanvasRenderingContext2D, shape: Shape<MarkerGeometry>): void {
  const { x, y, radius, heading } = shape.geometry

  const headingAngle = Math.atan2(heading.y, heading.x)
  const arcDistance = 3 * Math.PI * 0.5

  // By default, the arc startAngle begins on the positive x-axis and rotates clockwise. If the startAngle and
  // endAngle are offset by a quadrant, the arc will point toward the positive x-axis instead of starting there.
  const startAngleOffset = Math.PI * 0.25
  const startAngle = headingAngle + startAngleOffset
  const endAngle = headingAngle + arcDistance + startAngleOffset

  ctx.beginPath()
  ctx.arc(x, y, radius, startAngle, endAngle)

  // The diagonal length of a unit square.
  const sqrt2 = Math.sqrt(2)

  // Convert the heading to absolute canvas coordinates.
  ctx.lineTo(x + sqrt2 * radius * heading.x, y + sqrt2 * radius * heading.y)
  ctx.closePath()

  applyAppearance(ctx, shape.appearance)

  if (shape.appearance.stroke) {
    ctx.stroke()
  }
  if (shape.appearance instanceof BasicAppearance && shape.appearance.fill) {
    ctx.fill()
  }
}
