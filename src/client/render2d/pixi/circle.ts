import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'

import { CircleGeometry } from '../geometry/circle_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderCircle = createTransformer((shape: Shape<CircleGeometry>): Graphics => {

  const { x, y, radius } = shape.geometry

  const g = new Graphics()

  applyAppearance(g, shape.appearance, g => {
    g.drawCircle(x, y, radius)
  })

  return g
})
