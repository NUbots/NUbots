import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'

import { LineGeometry } from '../geometry/line_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderLine = createTransformer((shape: Shape<LineGeometry>): Graphics => {

  const { origin, target } = shape.geometry

  const g = new Graphics()

  applyAppearance(g, shape.appearance, g => {
    g.moveTo(origin.x, origin.y)
    g.lineTo(target.x, target.y)
  })

  return g
})
