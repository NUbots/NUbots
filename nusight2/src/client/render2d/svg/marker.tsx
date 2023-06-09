import React, { useContext } from "react";
import { observer } from "mobx-react";

import { Transform } from "../../../shared/math/transform";
import { Vector2 } from "../../../shared/math/vector2";
import { MarkerGeometry } from "../geometry/marker_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<MarkerGeometry> };
export const Marker = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const { x, y, radius, heading } = geometry;
  const transform = useContext(rendererTransformsContext);

  // We only want the direction from the heading
  const headingN = heading.normalize();
  const arcStartDir = headingN.transform(Transform.of({ rotate: Math.PI * 0.25 }));
  const arcStart = Vector2.of(x + arcStartDir.x * radius, y + arcStartDir.y * radius);

  const arcEndDir = headingN.transform(Transform.of({ rotate: -Math.PI * 0.25 }));
  const arcEnd = Vector2.of(x + arcEndDir.x * radius, y + arcEndDir.y * radius);

  const sqrt2 = Math.sqrt(2);
  const markerPoint = Vector2.of(x + sqrt2 * radius * headingN.x, y + sqrt2 * radius * headingN.y);

  let path = `M${arcStart.x},${arcStart.y}`;
  path += `A${radius},${radius},270,1,1,${arcEnd.x},${arcEnd.y}`;
  path += `L${markerPoint.x},${markerPoint.y}`;
  path += "Z";
  return <path d={path} {...toSvgAppearance(appearance)} {...toSvgEventHandlers(eventHandlers, transform)} />;
});
