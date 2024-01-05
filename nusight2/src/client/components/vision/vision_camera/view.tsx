import React, { Component } from "react";
import { action, computed } from "mobx";
import { observer } from "mobx-react";

import { CameraView } from "../../camera/view";
import { SwitchesMenuOption } from "../../switches_menu/view";
import { VisionRobotModel } from "../model";

import { VisionCameraModel } from "./model";
import { VisionCameraViewModel } from "./view_model";

export interface VisionCameraViewProps {
  model: VisionCameraModel;
  viewModel: VisionCameraViewModel;
  robot: VisionRobotModel;
  viewType: "full" | "thumbnail";
  objectFit: "fill" | "contain" | "cover";
  allowPanAndZoom: boolean;
  onClick?({ button }: { button: number }): void;
}

@observer
export class VisionCameraView extends Component<VisionCameraViewProps> {
  static defaultProps = {
    viewType: "full",
    objectFit: "contain",
    allowPanAndZoom: false,
  } as const;

  render() {
    const { viewModel, viewType, objectFit, allowPanAndZoom, onClick } = this.props;
    return (
      <CameraView
        viewModel={viewModel}
        switchMenuOptions={this.switchMenuOptions}
        viewType={viewType}
        objectFit={objectFit}
        allowPanAndZoom={allowPanAndZoom}
        onClick={onClick}
      />
    );
  }

  @computed
  private get switchMenuOptions(): SwitchesMenuOption[] {
    const drawOptions = this.props.viewModel.drawOptions;
    return [
      {
        label: "Visual Mesh",
        enabled: drawOptions.drawVisualMesh,
        toggle: action(() => (drawOptions.drawVisualMesh = !drawOptions.drawVisualMesh)),
      },
      {
        label: "Green Horizon",
        enabled: drawOptions.drawGreenHorizon,
        toggle: action(() => (drawOptions.drawGreenHorizon = !drawOptions.drawGreenHorizon)),
      },
      {
        label: "Balls",
        enabled: drawOptions.drawBalls,
        toggle: action(() => (drawOptions.drawBalls = !drawOptions.drawBalls)),
      },
      {
        label: "Goals",
        enabled: drawOptions.drawGoals,
        toggle: action(() => (drawOptions.drawGoals = !drawOptions.drawGoals)),
      },
      {
        label: "Robots",
        enabled: drawOptions.drawRobots,
        toggle: action(() => (drawOptions.drawRobots = !drawOptions.drawRobots)),
      },
    ];
  }
}
