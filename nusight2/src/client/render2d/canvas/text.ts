import { Transform } from '../../math/transform'
import { Vector2 } from '../../math/vector2'
import { TextGeometry } from '../geometry/text_geometry'
import { Shape } from '../object/shape'

import { applyAppearance } from './rendering'

export function renderText(
  ctx: CanvasRenderingContext2D,
  shape: Shape<TextGeometry>,
  world: Transform,
): void {
  const { x, y, text, fontSize, fontFamily, textAlign, textBaseline, worldAlignment, worldScale } =
    shape.geometry

  ctx.font = `${fontSize} ${fontFamily}`
  ctx.textAlign = textAlign === 'middle' ? 'center' : textAlign
  ctx.textBaseline = textBaseline

  const position = Vector2.of(x, y)

  if (worldScale) {
    // Ensure the text is always rendered without rotation such that it is aligned with the screen.
    ctx.scale(1 / world.scale.x, 1 / world.scale.y)
  }

  if (worldAlignment) {
    ctx.rotate(-world.rotate)
  }

  ctx.translate(position.x, position.y)

  applyAppearance(ctx, shape.appearance)

  ctx.fillText(text, 0, 0)
}
