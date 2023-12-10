import { action } from "mobx";

import { RobotModel } from "../robot/model";

import { ProfilerRobotModel } from "./model";
import { ProfilerModel } from "./model";
import { ProfileSort } from "./model";

export class ProfilerController {
  static of() {
    return new ProfilerController();
  }

  onSelectRobot(model: ProfilerModel, robot?: RobotModel) {
    model.selectedRobot = robot && ProfilerRobotModel.of(robot);
  }

  @action.bound
  setSort(model: ProfilerRobotModel, column: ProfileSort["column"]) {
    if (model.sortProfile.column === column) {
      model.sortProfile.direction = model.sortProfile.direction === "asc" ? "desc" : "asc";
    } else {
      model.sortProfile.column = column;
      model.sortProfile.direction = "asc";
    }
  }

  @action.bound
  setSearch(model: ProfilerRobotModel, search: string) {
    model.search = search;
  }

  getSortIcon(model: ProfilerRobotModel, column: string) {
    if (model.sortProfile.column === column) {
      return model.sortProfile.direction === "asc" ? "↑" : "↓";
    }
    return "";
  }

  getPercentageStyle(percentage: number) {
    // Clamp the percentage between 0 and 100
    const clampedPercentage = Math.min(Math.max(percentage, 0), 100);
    return {
      width: `${clampedPercentage}%`,
      backgroundColor: "lightblue",
      height: "100%",
    };
  }
}
