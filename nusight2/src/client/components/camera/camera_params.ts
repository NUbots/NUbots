import { observable } from "mobx";

import { Matrix4 } from "../../../shared/math/matrix4";

import { Lens } from "./lens";

export class CameraParams {
  @observable.ref accessor Hcw: Matrix4;
  @observable.ref accessor lens: Lens;

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
