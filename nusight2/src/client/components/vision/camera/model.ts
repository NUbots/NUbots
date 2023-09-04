import { observable } from "mobx";

import { Vector2 } from "../../../../shared/math/vector2";

import { CameraParams } from "./camera_params";
import { Image } from "./image";
import { BallModel } from "./objects/balls";
import { GoalModel } from "./objects/goals";
import { GreenHorizonModel } from "./objects/green_horizon";
import { VisualMeshModel } from "./objects/visual_mesh";

export interface CameraDefaultDrawOptions {
  drawImage: boolean;
  drawDistance: boolean;
  drawCompass: boolean;
  drawHorizon: boolean;
}

export interface CameraModel {
  name: string;
  image: Image;
  params: CameraParams;
  drawOptions: CameraDefaultDrawOptions;
}

type DrawOptions = CameraDefaultDrawOptions & {
  drawVisualMesh: boolean;
  drawGreenHorizon: boolean;
  drawBalls: boolean;
  drawGoals: boolean;
};

export interface VisionCameraModelOpts {
  id: number;
  name: string;
  image: Image;
  params: CameraParams;
  visualMesh?: VisualMeshModel;
  greenHorizon?: GreenHorizonModel;
  balls?: BallModel[];
  goals?: GoalModel[];
  selected?: boolean;
}

export class VisionCameraModel implements CameraModel {
  @observable.ref id: number;
  @observable.ref selected: boolean;
  @observable.ref name: string;
  @observable.ref image: Image;
  @observable.ref params: CameraParams;

  @observable.ref visualMesh?: VisualMeshModel;
  @observable.ref greenHorizon?: GreenHorizonModel;
  @observable.ref balls?: BallModel[];
  @observable.ref goals?: GoalModel[];

  @observable drawOptions: DrawOptions;

  @observable zoom = 1;
  @observable pan = new Vector2(0, 0);
  @observable isPanning: boolean = false;

  constructor(opts: VisionCameraModelOpts & { drawOptions: DrawOptions }) {
    this.id = opts.id;
    this.name = opts.name;
    this.image = opts.image;
    this.params = opts.params;
    this.visualMesh = opts.visualMesh;
    this.greenHorizon = opts.greenHorizon;
    this.balls = opts.balls;
    this.goals = opts.goals;
    this.drawOptions = opts.drawOptions;
    this.selected = opts.selected ?? false;
  }

  static of(opts: VisionCameraModelOpts) {
    return new VisionCameraModel({
      ...opts,
      drawOptions: {
        drawImage: true,
        drawDistance: false,
        drawCompass: true,
        drawHorizon: true,
        drawVisualMesh: false,
        drawGreenHorizon: true,
        drawBalls: true,
        drawGoals: true,
      },
    });
  }

  copy(that: VisionCameraModel) {
    this.id = that.id;
    this.name = that.name;
    this.image = that.image;
    this.params.copy(that.params);
    this.visualMesh = that.visualMesh;
    this.greenHorizon = (that.greenHorizon && this.greenHorizon?.copy(that.greenHorizon)) || that.greenHorizon;
    this.balls = that.balls;
    this.goals = that.goals;
    return this;
  }
}
