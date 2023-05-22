import React, { useContext } from "react";
import { observer } from "mobx-react";

import { PolygonGeometry } from "../geometry/polygon_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<PolygonGeometry> };
export const Polygon = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const transforms = useContext(rendererTransformsContext);
  return (
    <polygon
      points={geometry.points.map((p) => `${p.x},${p.y}`).join(" ")}
      {...toSvgAppearance(appearance)}
      {...toSvgEventHandlers(eventHandlers, transforms)}
    />
  );
});
