import { computed } from "mobx";
import { observable } from "mobx";

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

export type ProfileSort = {
  column: "name" | "reactionId" | "reactor" | "total_time" | "count" | "min_time" | "max_time" | "avg_time" | "percentage";
  direction: "asc" | "desc";
};

export class ProfilerRobotModel {
  robotModel: RobotModel;
  @observable profiles: Profile[] = [];
  @observable sortProfile: ProfileSort;

  constructor(robotModel: RobotModel) {
    this.robotModel = robotModel;
    this.sortProfile = {
      column: "name",
      direction: "asc",
    };
  }

  static of = memoize((robotModel: RobotModel) => {
    return new ProfilerRobotModel(robotModel);
  });

  @computed
  get sortedProfiles() {
    return this.profiles.slice().sort((a, b) => {
      if (a[this.sortProfile.column] < b[this.sortProfile.column]) {
        return this.sortProfile.direction === "asc" ? -1 : 1;
      }
      if (a[this.sortProfile.column] > b[this.sortProfile.column]) {
        return this.sortProfile.direction === "asc" ? 1 : -1;
      }
      return 0;
    });
  }
}

export class Profile {
  @observable name: string;
  @observable reactionId: number;
  @observable reactor: string;
  @observable total_time: number;
  @observable count: number;
  @observable min_time: number;
  @observable max_time: number;
  @observable avg_time: number;
  @observable percentage: number;

  constructor(
    name: string,
    reactionId: number,
    reactor: string,
    total_time = 0,
    count = 0,
    min_time = Number.MAX_VALUE,
    max_time = 0,
    avg_time = 0,
    percentage = 0
  ) {
    this.name = name;
    this.reactionId = reactionId;
    this.reactor = reactor;
    this.total_time = total_time;
    this.count = count;
    this.min_time = min_time;
    this.max_time = max_time;
    this.avg_time = avg_time;
    this.percentage = percentage;
  }

  updateProfile(newTime: number, total_time_all: number) {
    this.total_time += newTime;
    this.count++;
    this.min_time = Math.min(this.min_time, newTime);
    this.max_time = Math.max(this.max_time, newTime);
    this.avg_time = this.total_time / this.count;
    this.percentage = 100.0 * this.total_time / total_time_all;
  }
}
