import React, { useContext, useMemo } from "react";
import { observer } from "mobx-react";

import { Group as GroupGeometry } from "../object/group";
import { rendererTransformsContext } from "../svg_renderer";

import { ShapeView, toSvgTransform } from "./rendering";

type Props = { model: GroupGeometry };
export const Group = observer(({ model: { children, transform } }: Props) => {
  const { camera, world, getSVGOffset } = useContext(rendererTransformsContext);
  const transforms = useMemo(
    () => ({ camera, getSVGOffset, world: world.then(transform) }),
    [camera, world, transform],
  );

  const elems = children.map((obj, i) => {
    return obj instanceof GroupGeometry ? <Group key={i} model={obj} /> : <ShapeView key={i} obj={obj} />;
  });

  return transform.isIdentity() ? (
    <>{elems}</> // If we have the identity transform forgo the group to save on dom elements
  ) : (
    <g transform={toSvgTransform(transform)}>
      {/*
        Update the world transform so that it transforms a coordinate in this group back to
        the equivalent world coordinate
      */}
      <rendererTransformsContext.Provider value={transforms}>{elems}</rendererTransformsContext.Provider>
    </g>
  );
});
