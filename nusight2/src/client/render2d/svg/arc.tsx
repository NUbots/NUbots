import React, { useContext } from "react";
import { observer } from "mobx-react";

import { Vector2 } from "../../../shared/math/vector2";
import { ArcGeometry } from "../geometry/arc_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<ArcGeometry> };
export const Arc = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const { origin, radius, startAngle, endAngle, anticlockwise } = geometry;

  // Is this arc empty?
  if (radius < 0) {
    throw new Error(`Negative radius: ${radius}`);
  }

  const transforms = useContext(rendererTransformsContext);

  // Calculate out our start and end points on the circle
  const p0 = Vector2.fromPolar(radius, startAngle).add(origin);
  const p1 = Vector2.fromPolar(radius, endAngle).add(origin);

  // Our cw value must be a 0/1 not true/false
  const cw = +anticlockwise;

  // Shortest rotation required to reach end angle from start angle
  // where the sign indicates the direction to rotate
  const da = ((endAngle - startAngle + Math.PI) % (2 * Math.PI)) - Math.PI;

  // Draw the large arc if the sign does not match the set direction
  const largeArcFlag = +((Math.sign(da) === 1) !== anticlockwise);

  // Move to (x0,y0).
  let path = `M${p0.x} ${p0.y}`;

  // Draw the arc
  path += `A${radius} ${radius} 0 ${largeArcFlag} ${cw} ${p1.x} ${p1.y}`;
  return <path d={path} {...toSvgAppearance(appearance)} {...toSvgEventHandlers(eventHandlers, transforms)} />;
});
