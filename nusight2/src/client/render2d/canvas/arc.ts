import { ArcGeometry } from "../geometry/arc_geometry";
import { Shape } from "../object/shape";

export function renderArc(ctx: CanvasRenderingContext2D, shape: Shape<ArcGeometry>): void {
  const { origin, radius, startAngle, endAngle, anticlockwise } = shape.geometry;

  ctx.beginPath();

  // We must swap the direction that the arc is drawn since the y-direction is flipped
  ctx.arc(origin.x, origin.y, radius, startAngle, endAngle, !anticlockwise);
}
