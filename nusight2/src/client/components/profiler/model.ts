import { computed } from "mobx";
import { observable } from "mobx";
import internal from "stream";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { RobotModel } from "../robot/model";

export class ProfilerModel {
  @observable.ref selectedRobot?: ProfilerRobotModel;

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new ProfilerModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((r) => r.enabled);
  }
}

export class ProfilerRobotModel {
  @observable profiles: Profile[] = [];

  constructor(readonly robotModel: RobotModel) {}

  static of = memoize((robotModel: RobotModel) => {
    return new ProfilerRobotModel(
      robotModel,
    );
  });

  // Method to update profiles
  updateProfiles(newProfiles: Profile[]) {
    this.profiles = newProfiles;
  }
}

export class Profile {
  constructor(
    readonly name: string,
    readonly total_time: number,
    readonly count: number,
    readonly min_time: number,
    readonly max_time: number,
    readonly avg_time: number
  ) {}
}
