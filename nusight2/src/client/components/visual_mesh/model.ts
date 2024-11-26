import { computed } from "mobx";
import { observable } from "mobx";

import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";
import { RobotModel } from "../robot/model";

import { CameraModel } from "./camera/model";

export class VisualMeshModel {
  constructor(private appModel: AppModel) {}

  static of = memoize((appModel: AppModel) => {
    return new VisualMeshModel(appModel);
  });

  @computed
  get robots(): VisualMeshRobotModel[] {
    return this.appModel.robots.map(VisualMeshRobotModel.of);
  }
}

export class VisualMeshRobotModel {
  @observable accessor cameras: Map<number, CameraModel> = new Map();

  constructor(private robotModel: RobotModel) {}

  static of = memoize((robotModel: RobotModel) => {
    return new VisualMeshRobotModel(robotModel);
  });

  @computed
  get id() {
    return this.robotModel.id;
  }

  @computed
  get name() {
    return this.robotModel.name;
  }

  @computed
  get visible() {
    return this.robotModel.enabled;
  }
}
