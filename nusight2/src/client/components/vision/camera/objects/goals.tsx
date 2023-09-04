import { observer } from 'mobx-react'
import React from "react";

import { UnreachableError } from "../../../../../shared/base/unreachable_error";
import { FieldDimensions } from "../../../../../shared/field/dimensions";
import { Matrix4 } from "../../../../../shared/math/matrix4";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { CameraParams } from "../camera_params";

import { PlaneSegmentView } from "./line_projection";

export interface GoalModel {
  readonly timestamp: number;
  readonly Hcw: Matrix4;
  readonly side: "left" | "right" | "unknown";
  readonly post: {
    readonly top: Vector3;
    readonly bottom: Vector3;
    readonly distance: number;
  };
}

export const GoalsView = observer(({
  model,
  params,
  imageAspectRatio,
}: {
  model: GoalModel[];
  params: CameraParams;
  imageAspectRatio: number;
}) => {
  return (
    <object3D>
      {model.map((goal, index) => {
        // Transform the goal line so that it is in the perspective of the latest camera image.
        const Hwc = goal.Hcw.invert();
        const Hcc = params.Hcw.multiply(Hwc);
        const bottom = goal.post.bottom.multiplyScalar(goal.post.distance);
        return (
          <PlaneSegmentView
            key={index}
            segment={{
              start: bottom
                .applyMatrix4(Hwc)
                .add(new Vector3(0, 0, FieldDimensions.of().goalCrossbarHeight))
                .applyMatrix4(params.Hcw)
                .normalize(),
              end: bottom.applyMatrix4(Hcc).normalize(),
              color: getColor(goal.side),
              lineWidth: 10,
            }}
            lens={params.lens}
            imageAspectRatio={imageAspectRatio}
          />
        );
      })}
    </object3D>
  );
});

function getColor(side: GoalModel["side"]) {
  switch (side) {
    case "left":
      return new Vector4(1.0, 1.0, 0, 1.0); // Yellow
    case "right":
      return new Vector4(0.0, 1.0, 1.0, 1.0); // Cyan
    case "unknown":
      return new Vector4(1.0, 0.0, 1.0, 1.0); // Magenta
    default:
      throw new UnreachableError(side);
  }
}
