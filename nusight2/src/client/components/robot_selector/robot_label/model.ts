import { observable } from "mobx";
import { computed } from "mobx";
import { createTransformer } from "mobx-utils";

import { RobotNetworkStatsModel } from "../../../network/model";
import { RobotModel } from "../../robot/model";

export class RobotLabelModel {
  @observable accessor statsOpen = false;

  constructor(private robotModel: RobotModel) {}

  static of = createTransformer((robotModel: RobotModel): RobotLabelModel => {
    return new RobotLabelModel(robotModel);
  });

  @computed
  get stats() {
    return RobotNetworkStatsModel.of(this.robotModel);
  }
}
