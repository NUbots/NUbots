import { BasicAppearance } from '../appearance/basic_appearance'
import { ArcGeometry } from '../geometry/arc_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export function renderArc(ctx: CanvasRenderingContext2D, shape: Shape<ArcGeometry>): void {
  const { origin, radius, startAngle, endAngle, anticlockwise } = shape.geometry

  ctx.beginPath()
  ctx.arc(origin.x, origin.y, radius, startAngle, endAngle, anticlockwise)

  applyAppearance(ctx, shape.appearance)

  if (shape.appearance.stroke) {
    ctx.stroke()
  }
  if (shape.appearance instanceof BasicAppearance && shape.appearance.fill) {
    ctx.fill()
  }
}
