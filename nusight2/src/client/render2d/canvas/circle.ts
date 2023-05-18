import { CircleGeometry } from "../geometry/circle_geometry";
import { Shape } from "../object/shape";

export function renderCircle(ctx: CanvasRenderingContext2D, shape: Shape<CircleGeometry>): void {
  const { x, y, radius } = shape.geometry;

  ctx.beginPath();
  ctx.arc(x, y, radius, 0, 2 * Math.PI);
}
