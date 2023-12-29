import { computed } from "mobx";

import { CameraViewModel } from "../../camera/view_model";
import { VisionRobotModel } from "../model";

import { BallsViewModel } from "./balls";
import { GoalsViewModel } from "./goals";
import { OtherRobotsViewModel } from "./other_robots";
import { GreenHorizonViewModel } from "./green_horizon";
import { VisionCameraModel } from "./model";
import { VisualMeshViewModel } from "./visual_mesh";

export class VisionCameraViewModel extends CameraViewModel {
  constructor(readonly model: VisionCameraModel, private robot: VisionRobotModel) {
    super(model);
  }

  static of(model: VisionCameraModel, robot: VisionRobotModel) {
    return new VisionCameraViewModel(model, robot);
  }

  readonly getRenderables = () => {
    const { drawOptions } = this.model;
    return [
      drawOptions.drawVisualMesh && this.visualMesh?.visualMesh(),
      drawOptions.drawGreenHorizon && this.greenHorizon?.greenHorizon(),
      drawOptions.drawBalls && this.balls?.balls(),
      drawOptions.drawGoals && this.goals?.goals(),
      drawOptions.drawRobots && this.robots?.robots(),
    ];
  };

  @computed
  get drawOptions() {
    return this.model.drawOptions;
  }

  @computed
  private get greenHorizon(): GreenHorizonViewModel | undefined {
    const { greenHorizon, params } = this.model;
    return greenHorizon && GreenHorizonViewModel.of(greenHorizon, params, this.canvas, this.imageAspectRatio);
  }

  @computed
  private get balls(): BallsViewModel | undefined {
    return (
      this.model.balls && BallsViewModel.of(this.model.balls, this.model.params, this.canvas, this.imageAspectRatio)
    );
  }

  @computed
  private get goals(): GoalsViewModel | undefined {
    return (
      this.model.goals && GoalsViewModel.of(this.model.goals, this.model.params, this.canvas, this.imageAspectRatio)
    );
  }

  @computed
  private get robots(): OtherRobotsViewModel | undefined {
    return (
      this.model.robots && OtherRobotsViewModel.of(this.model.robots, this.model.params, this.canvas, this.imageAspectRatio)
    );
  }

  @computed
  private get visualMesh(): VisualMeshViewModel | undefined {
    const { visualMesh, params } = this.model;
    return visualMesh && VisualMeshViewModel.of(visualMesh, params, this.canvas, this.imageAspectRatio);
  }
}
