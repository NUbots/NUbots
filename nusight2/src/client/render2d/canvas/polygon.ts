import { PolygonGeometry } from "../geometry/polygon_geometry";
import { Shape } from "../object/shape";

export function renderPolygon(ctx: CanvasRenderingContext2D, shape: Shape<PolygonGeometry>): void {
  const { points } = shape.geometry;

  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);

  for (const point of points.slice(0)) {
    ctx.lineTo(point.x, point.y);
  }

  ctx.closePath();
}
