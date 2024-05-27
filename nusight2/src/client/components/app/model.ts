import { observable } from "mobx";

import { NbsScrubbersModel } from "../nbs_scrubbers/model";
import { RobotModel } from "../robot/model";

interface AppModelOpts {
  robots: RobotModel[];
  scrubbersModel: NbsScrubbersModel;
}

export class AppModel {
  @observable robots: RobotModel[];
  readonly scrubbersModel: NbsScrubbersModel;

  constructor({ robots, scrubbersModel }: AppModelOpts) {
    this.robots = robots;
    this.scrubbersModel = scrubbersModel;
  }

  static of(opts?: Partial<AppModelOpts>) {
    return new AppModel({ robots: opts?.robots ?? [], scrubbersModel: opts?.scrubbersModel ?? NbsScrubbersModel.of() });
  }
}
