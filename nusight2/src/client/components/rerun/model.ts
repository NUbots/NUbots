import { observable } from "mobx";

import { RobotModel } from "../robot/model";

type RerunModelOpts = {
  robotModels: RobotModel[];
};

export class RerunModel {
  @observable private robotModels: RobotModel[];

  constructor({ robotModels }: { robotModels: RobotModel[] }) {
    this.robotModels = robotModels;
  }

  static of({ robotModels }: RerunModelOpts) {
    return new RerunModel({
      robotModels,
    });
  }
}
