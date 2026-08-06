import React, { useContext } from "react";
import { observer } from "mobx-react";

import { RectangleGeometry } from "../geometry/rectangle_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<RectangleGeometry> };
export const Rectangle = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const transforms = useContext(rendererTransformsContext);
  const radius =
    geometry.borderRadius === "full"
      ? Math.min(Math.abs(geometry.width), Math.abs(geometry.height)) / 2
      : geometry.borderRadius;
  return (
    <rect
      x={geometry.x}
      y={geometry.y}
      width={geometry.width}
      height={geometry.height}
      rx={radius?.toString()}
      {...toSvgAppearance(appearance)}
      {...toSvgEventHandlers(eventHandlers, transforms)}
    />
  );
});
