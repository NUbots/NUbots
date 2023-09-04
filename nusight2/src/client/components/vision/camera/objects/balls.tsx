import { observer } from 'mobx-react'
import React from "react";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { CameraParams } from "../camera_params";

import { ConeView } from "./line_projection";

interface Cone {
  readonly axis: Vector3;
  readonly radius: number;
}

export interface BallModel {
  readonly timestamp: number;
  readonly Hcw: Matrix4;
  readonly cone: Cone;
  readonly distance: number;
  readonly colour: Vector4;
}

export const BallsView = observer(({
  model,
  params,
  imageAspectRatio,
}: {
  model: BallModel[];
  params: CameraParams;
  imageAspectRatio: number;
}) => {
  return (
    <object3D>
      {model.map((ball, i) => {
        const Hwc = ball.Hcw.invert();
        const Hcc = params.Hcw.multiply(Hwc);
        // Transform the cone so that it is in the perspective of the latest camera image.
        const { axis, radius } = transform(ball.cone, ball.distance, Hcc);
        return (
          <ConeView
            key={i}
            segment={{
              axis,
              radius,
              color: ball.colour,
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

export function transform(cone: Cone, distance: number, transform: Matrix4): Cone {
  const oldPoint = cone.axis.multiplyScalar(distance);
  const newPoint = oldPoint.applyMatrix4(transform);
  const newDistanceSqr = newPoint.dot(newPoint);
  return {
    axis: newPoint.normalize(),
    // Source: https://en.wikipedia.org/wiki/Angular_diameter#Formula
    // Takes `radius = cos(asin(radiusActual / distance))` and solves for the new radius given a new distance.
    // Solve sin(acos(r_1)) * d_1 = sin(acos(r_2)) * d_2 for r_2
    // Simplifies to:
    radius: Math.sqrt(distance ** 2 * cone.radius ** 2 - distance ** 2 + newDistanceSqr) / Math.sqrt(newDistanceSqr),
  };
}
