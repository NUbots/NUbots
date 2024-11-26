import { observable } from "mobx";

import { Projection } from "../../../shared/math/projection";
import { Vector2 } from "../../../shared/math/vector2";

export class Lens {
  @observable.ref accessor projection: Projection;
  @observable.ref accessor focalLength: number;
  @observable.ref accessor centre: Vector2;
  @observable.ref accessor distortionCoeffecients: Vector2;

  constructor({
    projection,
    focalLength,
    centre = Vector2.of(0, 0),
    distortionCoeffecients = Vector2.of(0, 0),
  }: {
    projection: Projection;
    focalLength: number;
    centre?: Vector2;
    distortionCoeffecients?: Vector2;
  }) {
    this.projection = projection;
    this.focalLength = focalLength;
    this.centre = centre;
    this.distortionCoeffecients = distortionCoeffecients;
  }

  copy(that: Lens) {
    this.projection = that.projection;
    this.focalLength = that.focalLength;
    this.centre = that.centre;
    this.distortionCoeffecients = that.distortionCoeffecients;
    return this;
  }
}
