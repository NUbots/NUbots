import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'
import { Point } from 'pixi.js'

import { PolygonGeometry } from '../geometry/polygon_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderPolygon = createTransformer((shape: Shape<PolygonGeometry>): Graphics => {
  const { points } = shape.geometry

  const polygon = points.slice(0)
  polygon.push(polygon[0])

  const g = new Graphics()

  applyAppearance(g, shape.appearance, g => {
    g.drawPolygon(polygon.map(p => new Point(p.x, p.y)))
  })

  return g
})
