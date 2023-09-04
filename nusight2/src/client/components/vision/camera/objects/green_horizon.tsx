import { observer } from 'mobx-react'
import React from "react";
import { observable } from "mobx";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { CameraParams } from "../camera_params";

import { PlaneSegmentView } from "./line_projection";

export class GreenHorizonModel {
  /** A list of world space camera unit-vector rays. */
  @observable.ref horizon: Vector3[];

  /** The world to camera transform, at the time the green horizon was measured. */
  @observable.ref Hcw: Matrix4;

  constructor({ horizon, Hcw }: { horizon?: Vector3[]; Hcw?: Matrix4 }) {
    this.horizon = horizon ?? [];
    this.Hcw = Hcw ?? Matrix4.of();
  }

  copy(that: GreenHorizonModel) {
    this.horizon = that.horizon;
    this.Hcw = that.Hcw;
    return this;
  }
}

export const GreenHorizonView = observer(({
  model,
  params,
  imageAspectRatio,
}: {
  model: GreenHorizonModel;
  params: CameraParams;
  imageAspectRatio: number;
}) => {
  const rays = React.useMemo(() => {
    // This method transforms the green horizon rays such that they track correctly with the latest camera image.
    //
    // e.g. If we have a 5 second old green horizon measurement, and receive a new camera image, the image may have
    // shifted perspective, and the green horizon would look incorrectly offset on screen.
    //
    // We transform the rays by using the Hcw from the green horizon measurement and the Hcw from the camera image
    // measurement. We firstly project the rays down to the ground using the height from the green horizon Hcw
    // We then transform the ground points into world space by adding the translation to that camera.
    // Once we have ground points in the world space, we use any Hcw matrix to transform them to that camera's view.
    // This effectively remaps the rays to the perspective of the new camera image.

    const { horizon, Hcw: greenHorizonHcw } = model;
    const imageHcw = params.Hcw;
    const greenHorizonHwc = greenHorizonHcw.invert();
    const rCWw = greenHorizonHwc.t.vec3();
    return horizon.map(
      (ray) =>
        ray // rUCw
          // Project world space unit vector onto the world/field ground, giving us a camera to field vector in world space.
          .multiplyScalar(ray.z !== 0 ? -greenHorizonHwc.t.z / ray.z : 1) // rFCw
          // Get the world to field vector, so that we can...
          .add(rCWw) // rFWw = rFCw + rCWw
          // ...apply the camera image's world to camera transform, giving us a corrected camera space vector.
          .applyMatrix4(imageHcw) // rFCc
          // Normalize to get the final camera space direction vector/ray.
          .normalize(), // rUCc
    );
  }, [model.horizon, model.Hcw, params.Hcw]);
  return (
    <object3D>
      {model.horizon.map((_, index) => {
        // For n given rays there are n - 1 line segments between them.
        return index >= 1 && rays[index - 1] && rays[index] ? (
          <PlaneSegmentView
            key={index}
            segment={{
              start: rays[index - 1],
              end: rays[index],
              color: new Vector4(0, 0.8, 0, 0.8),
              lineWidth: 10,
            }}
            lens={params.lens}
            imageAspectRatio={imageAspectRatio}
          />
        ) : null;
      })}
    </object3D>
  );
});
