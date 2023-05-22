import React, { useContext } from "react";
import { observer } from "mobx-react";

import { LineGeometry } from "../geometry/line_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<LineGeometry> };
export const Line = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const transforms = useContext(rendererTransformsContext);
  return (
    <line
      x1={geometry.origin.x}
      y1={geometry.origin.y}
      x2={geometry.target.x}
      y2={geometry.target.y}
      {...toSvgAppearance(appearance)}
      {...toSvgEventHandlers(eventHandlers, transforms)}
    />
  );
});
