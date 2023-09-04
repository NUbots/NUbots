// Compass and horizon and distance circles are ENU

import { observer } from 'mobx-react'
import React from "react";

import { Vector4 } from "../../../../../shared/math/vector4";
import { CameraParams } from "../camera_params";

import { PlaneSegmentView } from "./line_projection";

export const CompassView = observer(({ params, imageAspectRatio }: { params: CameraParams; imageAspectRatio: number }) => {
  const lineWidth = 5;
  return (
    <object3D>
      <PlaneSegmentView
        segment={{
          // xPositiveAxis
          start: params.Hcw.x.vec3(),
          end: params.Hcw.z.vec3().multiplyScalar(-1),
          color: new Vector4(1, 0, 0, 0.5), // Red
          lineWidth,
        }}
        lens={params.lens}
        imageAspectRatio={imageAspectRatio}
      />
      <PlaneSegmentView
        segment={{
          // xNegativeAxis
          start: params.Hcw.x.vec3().multiplyScalar(-1),
          end: params.Hcw.z.vec3().multiplyScalar(-1),
          color: new Vector4(0, 1, 1, 0.5), // Cyan
          lineWidth,
        }}
        lens={params.lens}
        imageAspectRatio={imageAspectRatio}
      />
      <PlaneSegmentView
        segment={{
          // yPositiveAxis
          start: params.Hcw.y.vec3(),
          end: params.Hcw.z.vec3().multiplyScalar(-1),
          color: new Vector4(0, 1, 0, 0.5), // Green
          lineWidth,
        }}
        lens={params.lens}
        imageAspectRatio={imageAspectRatio}
      />
      <PlaneSegmentView
        segment={{
          // yNegativeAxis
          start: params.Hcw.y.vec3().multiplyScalar(-1),
          end: params.Hcw.z.vec3().multiplyScalar(-1),
          color: new Vector4(1, 0, 1, 0.5), // Magenta
          lineWidth,
        }}
        lens={params.lens}
        imageAspectRatio={imageAspectRatio}
      />
    </object3D>
  );
});
