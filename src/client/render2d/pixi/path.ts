import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'
import { Point } from 'pixi.js'

import { PathGeometry } from '../geometry/path_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderPath = createTransformer((shape: Shape<PathGeometry>): Graphics => {

  const { points } = shape.geometry

  const g = new Graphics()

  applyAppearance(g, shape.appearance, g => {
    g.drawPolygon(points.map(p => new Point(p.x, p.y)))
  })

  return g
})
