import { observable } from "mobx";

import { Projection } from "../../../shared/math/projection";
import { Vector2 } from "../../../shared/math/vector2";

export class Lens {
  @observable.ref projection: Projection;
  @observable.ref focalLength: number;
  @observable.ref centre: Vector2;
  @observable.ref distortionCoefficients: Vector2;

  constructor({
    projection,
    focalLength,
    centre = Vector2.of(0, 0),
    distortionCoefficients = Vector2.of(0, 0),
  }: {
    projection: Projection;
    focalLength: number;
    centre?: Vector2;
    distortionCoefficients?: Vector2;
  }) {
    this.projection = projection;
    this.focalLength = focalLength;
    this.centre = centre;
    this.distortionCoefficients = distortionCoefficients;
  }

  copy(that: Lens) {
    this.projection = that.projection;
    this.focalLength = that.focalLength;
    this.centre = that.centre;
    this.distortionCoefficients = that.distortionCoefficients;
    return this;
  }
}
