import { observable } from "mobx";

import { CameraParams } from "../../camera/camera_params";
import { Image } from "../../camera/image";
import { CameraDefaultDrawOptions, CameraModel } from "../../camera/model";

import { BallModel } from "./balls";
import { BoundingBoxesModel } from "./bounding_boxes";
import { GoalModel } from "./goals";
import { GreenHorizonModel } from "./green_horizon";
import { OtherRobotsModel } from "./other_robots";
import { VisualMeshModel } from "./visual_mesh";

type DrawOptions = CameraDefaultDrawOptions & {
  drawVisualMesh: boolean;
  drawGreenHorizon: boolean;
  drawBalls: boolean;
  drawGoals: boolean;
  drawRobots: boolean;
  drawBoundingBoxes: boolean;
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
  robots?: OtherRobotsModel[];
  boundingBoxes?: BoundingBoxesModel[];
  selected?: boolean;
}

export class VisionCameraModel implements CameraModel {
  @observable.ref accessor id: number;
  @observable.ref accessor selected: boolean;
  @observable.ref accessor name: string;
  @observable.ref accessor image: Image;
  @observable.ref accessor params: CameraParams;

  @observable.ref accessor visualMesh?: VisualMeshModel;
  @observable.ref accessor greenHorizon?: GreenHorizonModel;
  @observable.ref accessor balls?: BallModel[];
  @observable.ref accessor goals?: GoalModel[];
  @observable.ref accessor robots?: OtherRobotsModel[];
  @observable.ref accessor boundingBoxes?: BoundingBoxesModel[];

  @observable accessor drawOptions: DrawOptions;

  constructor(opts: VisionCameraModelOpts & { drawOptions: DrawOptions }) {
    this.id = opts.id;
    this.name = opts.name;
    this.image = opts.image;
    this.params = opts.params;
    this.visualMesh = opts.visualMesh;
    this.greenHorizon = opts.greenHorizon;
    this.balls = opts.balls;
    this.goals = opts.goals;
    this.robots = opts.robots;
    this.boundingBoxes = opts.boundingBoxes;
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
        drawRobots: true,
        drawBoundingBoxes: true,
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
    this.robots = that.robots;
    this.boundingBoxes = that.boundingBoxes;
    return this;
  }
}
