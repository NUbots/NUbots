import { Transform } from "../../../shared/math/transform";
import { TextGeometry } from "../geometry/text_geometry";
import { Shape } from "../object/shape";

import { applyAppearance } from "./rendering";

export function renderText(ctx: CanvasRenderingContext2D, shape: Shape<TextGeometry>, world: Transform): void {
  const { x, y, text, fontSize, fontFamily, textAlign, textBaseline, worldAlignment, worldScale } = shape.geometry;

  ctx.font = `${fontSize} ${fontFamily}`;
  ctx.textAlign = textAlign === "middle" ? "center" : textAlign;
  ctx.textBaseline = textBaseline;

  ctx.translate(x, y);

  // Remove the world scaling but keep the dpi scaling so the text size matches what
  // it would be in an html element.
  if (worldScale) {
    ctx.scale(devicePixelRatio / world.scale.x, devicePixelRatio / world.scale.y);
  }

  if (worldAlignment) {
    ctx.rotate(world.rotate);
  }

  applyAppearance(ctx, shape.appearance);
  ctx.fillText(text, 0, 0);
}
