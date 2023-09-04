import { observer } from 'mobx-react'
import React from "react";

import { Vector4 } from "../../../../../shared/math/vector4";
import { CameraParams } from "../camera_params";

import { PlaneView } from "./line_projection";

export const HorizonView = observer(({ params, imageAspectRatio }: { params: CameraParams; imageAspectRatio: number }) => (
  <PlaneView
    axis={params.Hcw.z.vec3()}
    color={new Vector4(0, 0, 1, 0.7)}
    lineWidth={10}
    lens={params.lens}
    imageAspectRatio={imageAspectRatio}
  />
));
