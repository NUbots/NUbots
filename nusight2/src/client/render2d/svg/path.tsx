import React, { useContext } from "react";
import { observer } from "mobx-react";

import { PathGeometry } from "../geometry/path_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<PathGeometry> };
export const Path = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const transforms = useContext(rendererTransformsContext);
  const start = geometry.points[0];
  const path = `M${start.x},${start.y}` + geometry.points.map((p) => `L${p.x},${p.y}`);

  return (
    <path d={path} {...toSvgAppearance(appearance)} {...toSvgEventHandlers(eventHandlers, transforms)} fill="none" />
  );
});
