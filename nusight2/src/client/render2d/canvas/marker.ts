import { MarkerGeometry } from "../geometry/marker_geometry";
import { Shape } from "../object/shape";

export function renderMarker(ctx: CanvasRenderingContext2D, shape: Shape<MarkerGeometry>): void {
  const { x, y, radius, heading } = shape.geometry;

  // We only want the direction from the heading
  const headingDir = heading.normalize();

  const headingAngle = Math.atan2(headingDir.y, headingDir.x);
  const arcDistance = 3 * Math.PI * 0.5;

  // By default, the arc startAngle begins on the positive x-axis and rotates clockwise. If the startAngle and
  // endAngle are offset by a quadrant, the arc will point toward the positive x-axis instead of starting there.
  const startAngleOffset = Math.PI * 0.25;
  const startAngle = headingAngle + startAngleOffset;
  const endAngle = headingAngle + arcDistance + startAngleOffset;

  ctx.beginPath();
  ctx.arc(x, y, radius, startAngle, endAngle);

  // The diagonal length of a unit square.
  const sqrt2 = Math.sqrt(2);

  // Convert the heading to absolute canvas coordinates.
  ctx.lineTo(x + sqrt2 * radius * headingDir.x, y + sqrt2 * radius * headingDir.y);
  ctx.closePath();
}
