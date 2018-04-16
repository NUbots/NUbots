import { BasicAppearance } from '../appearance/basic_appearance'
import { PolygonGeometry } from '../geometry/polygon_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export function renderPolygon(ctx: CanvasRenderingContext2D, shape: Shape<PolygonGeometry>): void {
  const { points } = shape.geometry

  ctx.beginPath()
  ctx.moveTo(points[0].x, points[0].y)

  for (const point of points.slice(0)) {
    ctx.lineTo(point.x, point.y)
  }

  ctx.closePath()

  applyAppearance(ctx, shape.appearance)

  if (shape.appearance.stroke) {
    ctx.stroke()
  }
  if (shape.appearance instanceof BasicAppearance && shape.appearance.fill) {
    ctx.fill()
  }
}
