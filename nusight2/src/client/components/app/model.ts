import { observable } from "mobx";

import { RobotModel } from "../robot/model";

export class AppModel {
  @observable robots: RobotModel[];

  constructor({ robots }: AppModel) {
    this.robots = robots;
  }

  static of(options: { robots: RobotModel[] } = { robots: [] }) {
    return new AppModel(options);
  }
}
