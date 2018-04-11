import { createTransformer } from 'mobx-utils'
import { Graphics } from 'pixi.js'

import { ArrowGeometry } from '../geometry/arrow_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export const renderArrow = createTransformer((shape: Shape<ArrowGeometry>): Graphics => {

  const { length, width, headLength, headWidth, origin, direction } = shape.geometry

  const w = width * 0.5
  const hl = headLength * 0.5
  const hw = headWidth * 0.5

  const g = new Graphics()
  g.x = origin.x
  g.y = origin.y
  g.rotation = Math.atan2(direction.y, direction.x)

  g.moveTo(0, -w)

  applyAppearance(g, shape.appearance, g => {
    g.lineTo(length - hl, -w)
    g.lineTo(length - hl, -hw)
    g.lineTo(length, 0)
    g.lineTo(length - hl, hw)
    g.lineTo(length - hl, w)
    g.lineTo(0, w)
    g.closePath()
  })

  return g

})
