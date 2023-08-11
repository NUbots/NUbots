import { action } from "mobx";

import { DashboardModel } from "./model";

export class DashboardController {
  static of() {
    return new DashboardController();
  }

  @action
  toggleOrientation(model: DashboardModel) {
    model.field.orientation = model.field.orientation === "left" ? "right" : "left";
  }
}
