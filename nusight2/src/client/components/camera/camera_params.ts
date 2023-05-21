import { observable } from "mobx";

import { Matrix4 } from "../../../shared/math/matrix4";

import { Lens } from "./lens";

export class CameraParams {
  @observable.ref Hcw: Matrix4;
  @observable.ref lens: Lens;

  constructor({ Hcw, lens }: { Hcw: Matrix4; lens: Lens }) {
    this.Hcw = Hcw;
    this.lens = lens;
  }

  copy(that: CameraParams) {
    this.Hcw = that.Hcw;
    this.lens.copy(that.lens);
    return this;
  }
}
