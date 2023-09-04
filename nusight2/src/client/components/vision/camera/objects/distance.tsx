import { observer } from 'mobx-react'
import React from "react";

import { range } from "../../../../../shared/base/range";
import { Vector4 } from "../../../../../shared/math/vector4";
import { CameraParams } from "../camera_params";

import { ConeView } from "./line_projection";

export const DistanceView = observer(({
  majorStep = 1,
  minorLines = 3,
  maxDistance = 50,
  params,
  imageAspectRatio,
}: {
  majorStep?: number;
  minorLines?: number;
  maxDistance?: number;
  params: CameraParams;
  imageAspectRatio: number;
}) => {
  const cameraHeight = React.useMemo(() => params.Hcw.invert().t.t, [params.Hcw]);
  return (
    <object3D>
      {range(((minorLines + 1) * maxDistance) / majorStep).map((i) => (
        <ConeView
          key={i}
          segment={{
            axis: params.Hcw.z.vec3().multiplyScalar(-1),
            radius: Math.cos(Math.atan((i + 1) / (minorLines + 1) / cameraHeight)),
            color: new Vector4(1, 1, 1, (i + 1) % (minorLines + 1) ? 0.2 : 0.4),
            lineWidth: (i + 1) % (minorLines + 1) ? 2 : 3,
          }}
          lens={params.lens}
          imageAspectRatio={imageAspectRatio}
        />
      ))}
    </object3D>
  );
});
