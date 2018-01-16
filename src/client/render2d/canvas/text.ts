import { Transform } from '../../math/transform'
import { Vector2 } from '../../math/vector2'
import { TextGeometry } from '../geometry/text_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export function renderText(ctx: CanvasRenderingContext2D, shape: Shape<TextGeometry>, world: Transform): void {

  const { x, y, text, maxWidth, fontFamily, textAlign, textBaseline, alignToView } = shape.geometry

  ctx.font = `1em ${fontFamily}`
  ctx.textAlign = textAlign === 'middle' ? 'center' : textAlign
  ctx.textBaseline = textBaseline

  const position = Vector2.of(x, y)

  const textWidth = ctx.measureText(text).width
  const scale = maxWidth / textWidth

  if (alignToView) {
    // Ensure the text is always rendered without rotation such that it is aligned with the screen.
    ctx.scale(Math.sign(world.scale.x), Math.sign(world.scale.y))
    ctx.rotate(-world.rotate)
    position.transform(Transform.of({
      rotate: -world.rotate,
      scale: { x: Math.sign(world.scale.x), y: Math.sign(world.scale.y) },
    }))
  }

  ctx.scale(scale, scale)
  ctx.translate(position.x / scale, position.y / scale)

  applyAppearance(ctx, shape.appearance)

  ctx.fillText(text, 0, 0)
}
