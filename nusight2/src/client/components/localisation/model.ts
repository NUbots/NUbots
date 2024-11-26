import { observable } from "mobx";
import { computed } from "mobx";

import { Vector3 } from "../../../shared/math/vector3";
import { memoize } from "../../base/memoize";
import { AppModel } from "../app/model";

import { FieldModel } from "./r3f_components/field/model";
import { SkyboxModel } from "./r3f_components/skybox/model";
import { LocalisationRobotModel } from "./robot_model";

export class TimeModel {
  @observable accessor time: number; // seconds
  @observable accessor lastPhysicsUpdate: number; // seconds

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
  @observable accessor position: Vector3;
  @observable accessor yaw: number;
  @observable accessor pitch: number;
  @observable accessor distance: number;

  constructor({ position, yaw, pitch, distance }: { position: Vector3; yaw: number; pitch: number; distance: number }) {
    this.position = position;
    this.yaw = yaw;
    this.pitch = pitch;
    this.distance = distance;
  }

  static of() {
    return new CameraModel({
      position: new Vector3(-1, 0, 1),
      yaw: 0,
      pitch: -Math.PI / 4,
      distance: 0.5,
    });
  }
}

export class ControlsModel {
  @observable accessor forward: boolean;
  @observable accessor left: boolean;
  @observable accessor right: boolean;
  @observable accessor back: boolean;
  @observable accessor up: boolean;
  @observable accessor down: boolean;
  @observable accessor pitch: number;
  @observable accessor yaw: number;

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
      pitch: 0,
      yaw: 0,
    });
  }
}

export class LocalisationModel {
  @observable private accessor appModel: AppModel;
  @observable accessor field: FieldModel;
  @observable accessor skybox: SkyboxModel;
  @observable accessor camera: CameraModel;
  @observable accessor locked: boolean;
  @observable accessor controls: ControlsModel;
  @observable accessor viewMode: ViewMode;
  @observable accessor target: LocalisationRobotModel | undefined;
  @observable accessor time: TimeModel;

  @observable accessor fieldVisible = true;
  @observable accessor gridVisible = true;
  @observable accessor robotVisible = true;
  @observable accessor ballVisible = true;
  @observable accessor particlesVisible = true;
  @observable accessor goalsVisible = true;
  @observable accessor fieldLinePointsVisible = true;
  @observable accessor fieldIntersectionsVisible = true;
  @observable accessor walkToDebugVisible = true;
  @observable accessor boundedBoxVisible = true;

  constructor(
    appModel: AppModel,
    {
      field,
      skybox,
      camera,
      locked,
      controls,
      viewMode,
      target,
      time,
    }: {
      field: FieldModel;
      skybox: SkyboxModel;
      camera: CameraModel;
      locked: boolean;
      controls: ControlsModel;
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
      viewMode: ViewMode.FreeCamera,
      time: TimeModel.of(),
    });
  });

  @computed get robots(): LocalisationRobotModel[] {
    return this.appModel.robots.map((robot) => LocalisationRobotModel.of(robot));
  }
}
