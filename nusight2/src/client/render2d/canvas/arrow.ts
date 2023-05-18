import { ArrowGeometry } from "../geometry/arrow_geometry";
import { Shape } from "../object/shape";

export function renderArrow(ctx: CanvasRenderingContext2D, shape: Shape<ArrowGeometry>): void {
  const { length, width, headLength, headWidth, origin, direction } = shape.geometry;

  const w = width * 0.5;
  const hl = headLength * 0.5;
  const hw = headWidth * 0.5;

  ctx.translate(origin.x, origin.y);
  ctx.rotate(Math.atan2(direction.y, direction.x));

  // Draw the arrow facing the positive x-axis.
  ctx.beginPath();
  ctx.moveTo(0, -w);
  ctx.lineTo(length - hl, -w);
  ctx.lineTo(length - hl, -hw);
  ctx.lineTo(length, 0);
  ctx.lineTo(length - hl, hw);
  ctx.lineTo(length - hl, w);
  ctx.lineTo(0, w);
  ctx.closePath();
}
