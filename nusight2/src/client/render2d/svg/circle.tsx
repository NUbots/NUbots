import React, { useContext } from "react";
import { observer } from "mobx-react";

import { CircleGeometry } from "../geometry/circle_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<CircleGeometry> };

export const Circle = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const transforms = useContext(rendererTransformsContext);
  return (
    <circle
      cx={geometry.x}
      cy={geometry.y}
      r={geometry.radius}
      {...toSvgAppearance(appearance)}
      {...toSvgEventHandlers(eventHandlers, transforms)}
    />
  );
});
