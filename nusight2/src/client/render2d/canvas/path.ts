import { PathGeometry } from '../geometry/path_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export function renderPath(ctx: CanvasRenderingContext2D, shape: Shape<PathGeometry>): void {
  const { points } = shape.geometry
  const start = points[0]

  ctx.beginPath()
  ctx.moveTo(start.x, start.y)
  points.forEach(p => ctx.lineTo(p.x, p.y))

  applyAppearance(ctx, shape.appearance)

  ctx.stroke()
}
