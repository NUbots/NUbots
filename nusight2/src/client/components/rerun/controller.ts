import { action } from "mobx";

import { RerunModel } from "./model";

export class RerunController {
  constructor(private model: RerunModel) {
    this.model = model;
  }

  static of(opts: { model: RerunModel }) {
    return new RerunController(opts.model);
  }
}
