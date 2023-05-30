import React, { useContext } from "react";
import { observer } from "mobx-react";

import { ArrowGeometry } from "../geometry/arrow_geometry";
import { Shape } from "../object/shape";
import { rendererTransformsContext } from "../svg_renderer";

import { toSvgAppearance, toSvgEventHandlers } from "./rendering";

type Props = { model: Shape<ArrowGeometry> };
export const Arrow = observer(({ model: { geometry, appearance, eventHandlers } }: Props) => {
  const { origin, direction, width, length, headWidth, headLength } = geometry;
  const transforms = useContext(rendererTransformsContext);

  const w = width * 0.5;
  const hl = headLength * 0.5;
  const hw = headWidth * 0.5;
  const r = (180 / Math.PI) * Math.atan2(direction.y, direction.x);

  let path = `M0 ${-w}`;
  path += `L${length - hl} ${-w}`;
  path += `L${length - hl} ${-hw}`;
  path += `L${length} 0`;
  path += `L${length - hl} ${hw}`;
  path += `L${length - hl} ${w}`;
  path += `L0 ${w}`;
  path += "Z";

  return (
    <path
      d={path}
      transform={`translate(${origin.x},${origin.y}) rotate(${r})`}
      {...toSvgAppearance(appearance)}
      {...toSvgEventHandlers(eventHandlers, transforms)}
    />
  );
});
