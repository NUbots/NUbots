import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'

import { ArcGeometry } from '../geometry/arc_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderArc = createTransformer((shape: Shape<ArcGeometry>): Graphics => {

  const { origin, radius, startAngle, endAngle, anticlockwise } = shape.geometry

  const g = new Graphics()

  applyAppearance(g, shape.appearance, g => {
    g.arc(origin.x, origin.y, radius, startAngle, endAngle, anticlockwise)
  })

  return g

})
