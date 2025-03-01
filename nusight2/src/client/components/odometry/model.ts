import { computed } from "mobx";
import { observable } from "mobx";

import { Vector3 } from "../../../shared/math/vector3";
import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { RobotModel } from "../robot/model";

import { OdometryVisualizerModel } from "./odometry_visualizer/model";

export class OdometryModel {
  @observable.ref selectedRobot?: OdometryRobotModel;

  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new OdometryModel(appModel);
  });

  @computed
  get robots(): RobotModel[] {
    return this.appModel.robots.filter((r) => r.enabled);
  }
}

export class OdometryRobotModel {
  constructor(
    readonly robotModel: RobotModel,
    readonly visualizerModel: OdometryVisualizerModel,
  ) {}

  static of = memoize((robotModel: RobotModel) => {
    return new OdometryRobotModel(
      robotModel,
      OdometryVisualizerModel.of({
        accelerometer: new Vector3(0, 0, -9.8),
      }),
    );
  });
}
