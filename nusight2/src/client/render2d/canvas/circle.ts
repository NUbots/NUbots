import { BasicAppearance } from '../appearance/basic_appearance'
import { CircleGeometry } from '../geometry/circle_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export function renderCircle(ctx: CanvasRenderingContext2D, shape: Shape<CircleGeometry>): void {
  const { x, y, radius } = shape.geometry

  ctx.beginPath()
  ctx.arc(x, y, radius, 0, 2 * Math.PI)

  applyAppearance(ctx, shape.appearance)

  if (shape.appearance.stroke) {
    ctx.stroke()
  }
  if (shape.appearance instanceof BasicAppearance && shape.appearance.fill) {
    ctx.fill()
  }
}
