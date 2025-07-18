import { observable } from "mobx";
import { computed } from "mobx";

import { Vector3 } from "../../../shared/math/vector3";
import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";

import { DashboardRobotModel } from "./dashboard_components/dashboard_robot/model";
import { DashboardFieldModel } from "./dashboard_components/field/model";
import { FieldModel } from "./r3f_components/field/model";
import { SkyboxModel } from "./r3f_components/skybox/model";
import { LocalisationRobotModel } from "./robot_model";

export class TimeModel {
  @observable time: number; // seconds
  @observable lastPhysicsUpdate: number; // seconds

  constructor({ time, lastPhysicsUpdate }: { time: number; lastPhysicsUpdate: number }) {
    this.time = time;
    this.lastPhysicsUpdate = lastPhysicsUpdate;
  }

  static of() {
    return new TimeModel({
      time: 0,
      lastPhysicsUpdate: 0,
    });
  }

  @computed get timeSinceLastPhysicsUpdate() {
    return this.time - this.lastPhysicsUpdate;
  }
}

export enum ViewMode {
  FreeCamera,
  FirstPerson,
  ThirdPerson,
}

class CameraModel {
  @observable position: Vector3;
  @observable yaw: number;
  @observable pitch: number;
  @observable distance: number;

  constructor({ position, yaw, pitch, distance }: { position: Vector3; yaw: number; pitch: number; distance: number }) {
    this.position = position;
    this.yaw = yaw;
    this.pitch = pitch;
    this.distance = distance;
  }

  static of() {
    return new CameraModel({
      position: new Vector3(0, 0, 5), // Hawk-eye view: center field, 5 units high
      yaw: Math.PI / 2, // Hawk-eye view yaw
      pitch: -Math.PI / 2, // Hawk-eye view: looking straight down
      distance: 0.5,
    });
  }
}

export class ControlsModel {
  @observable forward: boolean;
  @observable left: boolean;
  @observable right: boolean;
  @observable back: boolean;
  @observable up: boolean;
  @observable down: boolean;
  @observable pitch: number;
  @observable yaw: number;

  constructor({ forward, left, right, back, up, down, pitch, yaw }: ControlsModel) {
    this.forward = forward;
    this.left = left;
    this.right = right;
    this.back = back;
    this.up = up;
    this.down = down;
    this.pitch = pitch;
    this.yaw = yaw;
  }

  static of() {
    return new ControlsModel({
      forward: false,
      left: false,
      right: false,
      back: false,
      up: false,
      down: false,
      pitch: -Math.PI / 2, // Hawk-eye view: looking straight down
      yaw: Math.PI / 2, // Hawk-eye view yaw
    });
  }
}

export class DashboardModel {
  @observable visible = false;

  constructor() {}

  static of(): DashboardModel {
    return new DashboardModel();
  }
}

export class LocalisationModel {
  @observable private appModel: AppModel;
  @observable field: FieldModel;
  @observable skybox: SkyboxModel;
  @observable camera: CameraModel;
  @observable locked: boolean;
  @observable controls: ControlsModel;
  @observable dashboard: DashboardModel;
  @observable viewMode: ViewMode;
  @observable target?: LocalisationRobotModel;
  @observable time: TimeModel;

  @observable fieldVisible = true;
  @observable gridVisible = true;
  @observable robotVisible = true;
  @observable ballVisible = true;
  @observable particlesVisible = true;
  @observable goalsVisible = true;
  @observable fieldLinePointsVisible = true;
  @observable fieldIntersectionsVisible = true;
  @observable walkToDebugVisible = false;
  @observable boundedBoxVisible = true;

  constructor(
    appModel: AppModel,
    {
      field,
      skybox,
      camera,
      locked,
      controls,
      dashboard,
      viewMode,
      target,
      time,
    }: {
      field: FieldModel;
      skybox: SkyboxModel;
      camera: CameraModel;
      locked: boolean;
      controls: ControlsModel;
      dashboard: DashboardModel;
      viewMode: ViewMode;
      target?: LocalisationRobotModel;
      time: TimeModel;
    },
  ) {
    this.appModel = appModel;
    this.field = field;
    this.skybox = skybox;
    this.camera = camera;
    this.locked = locked;
    this.controls = controls;
    this.dashboard = dashboard;
    this.viewMode = viewMode;
    this.target = target;
    this.time = time;
  }

  static of = memoize((appModel: AppModel): LocalisationModel => {
    return new LocalisationModel(appModel, {
      field: FieldModel.of(),
      skybox: SkyboxModel.of(),
      camera: CameraModel.of(),
      locked: false,
      controls: ControlsModel.of(),
      dashboard: DashboardModel.of(),
      viewMode: ViewMode.FreeCamera,
      time: TimeModel.of(),
    });
  });

  @computed get robots(): LocalisationRobotModel[] {
    return this.appModel.robots.map((robot) => LocalisationRobotModel.of(robot));
  }

  @computed get dashboardRobots(): DashboardRobotModel[] {
    return this.appModel.robots.map((robot) => DashboardRobotModel.of(robot));
  }

  @computed get dashboardField(): DashboardFieldModel {
    return DashboardFieldModel.of(this.dashboardRobots);
  }
}
