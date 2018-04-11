import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'

import { PathGeometry } from '../geometry/path_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderPath = createTransformer((shape: Shape<PathGeometry>): Graphics => {

  const { points } = shape.geometry
  const start = points[0]

  const g = new Graphics()

  applyAppearance(g, shape.appearance, g => {
    g.moveTo(start.x, start.y)
    points.forEach(p => g.lineTo(p.x, p.y))
  })

  return g
})
