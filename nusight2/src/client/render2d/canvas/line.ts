import { LineGeometry } from "../geometry/line_geometry";
import { Shape } from "../object/shape";

export function renderLine(ctx: CanvasRenderingContext2D, shape: Shape<LineGeometry>): void {
  const { origin, target } = shape.geometry;

  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(target.x, target.y);
}
