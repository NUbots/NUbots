import { PropsWithChildren } from "react";
import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import { autorun, computed } from "mobx";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { VisionCameraModel } from "./camera/model";
import { VisionCameraViewProps } from "./camera/view";
import { VisionCameraView } from "./camera/view";
import { CameraImageViewer } from "./camera_image_viewer/view";
import { VisionController } from "./controller";
import { GridLayout } from "./grid_layout/grid_layout";
import { VisionModel, VisionRobotModel } from "./model";

@observer
export class VisionView extends Component<{
  controller: VisionController;
  model: VisionModel;
  Menu: ComponentType<PropsWithChildren>;
  // The camera view can optionally be replaced by a custom component,
  // which we do in one of the vision stories.
  CameraView?: ComponentType<VisionCameraViewProps>;
}> {
  private disposeRobotSelectAutorun?: () => void;

  componentDidMount() {
    if (!this.disposeRobotSelectAutorun) {
      this.disposeRobotSelectAutorun = autorun((reaction) => {
        // Automatically select the first available robot
        if (!this.props.model.selectedRobot && this.props.model.robots.length > 0) {
          this.onSelectRobot(this.props.model.robots[0]);

          // Clean up after the first run
          if (this.disposeRobotSelectAutorun) {
            reaction.dispose();
            this.disposeRobotSelectAutorun = undefined;
          }
        }
      });
    }
  }

  componentWillUnmount() {
    if (this.disposeRobotSelectAutorun) {
      this.disposeRobotSelectAutorun();
      this.disposeRobotSelectAutorun = undefined;
    }
  }

  render() {
    const {
      model: { selectedRobot, selectedCamera, robots },
      Menu,
    } = this.props;
    const CameraViewComponent = this.props.CameraView ?? VisionCameraView;
    return (
      <div className="flex-grow flex-shrink flex flex-col text-center relative w-full h-full">
        <Menu>
          <div className="h-full flex items-center justify-end">
            <RobotSelectorSingle robots={robots} selected={selectedRobot?.robotModel} onSelect={this.onSelectRobot} />
          </div>
        </Menu>
        {selectedRobot && (
          <div className="flex-grow">
            {selectedCamera ? (
              <CameraImageViewer
                key={selectedRobot.id}
                CameraView={CameraViewComponent}
                selectedRobot={selectedRobot}
                selectedCamera={selectedCamera}
                onSelectRobot={this.onSelectRobot}
                onSelectCamera={this.onSelectCamera}
                onPreviousCamera={this.onPreviousCamera}
                onNextCamera={this.onNextCamera}
              />
            ) : (
              <GridLayout itemAspectRatio={this.itemAspectRatio}>
                {selectedRobot?.cameraList.map((camera, index) => (
                  <GridCameraView
                    key={`${selectedRobot.id}_${camera.id}`}
                    CameraView={CameraViewComponent}
                    camera={camera}
                    selectedRobot={selectedRobot}
                    onClick={() => this.onSelectCamera(index)}
                  />
                ))}
              </GridLayout>
            )}
          </div>
        )}
      </div>
    );
  }

  @computed
  private get itemAspectRatio(): number {
    const {
      model: { selectedRobot },
    } = this.props;
    // Assumes aspect ratio for all cameras are the same, grab the first.
    const firstCamera: VisionCameraModel | undefined = selectedRobot?.cameras.values().next().value;
    return firstCamera ? firstCamera.image.width / firstCamera.image.height : 1;
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }

  @action.bound
  private onSelectCamera(cameraIndex: number) {
    this.props.controller.onSelectCamera(this.props.model, cameraIndex);
  }

  @action.bound
  private onPreviousCamera() {
    const newCameraIndex = Math.max(0, this.props.model.selectedCameraIndex - 1);
    this.props.controller.onSelectCamera(this.props.model, newCameraIndex);
  }

  @action.bound
  private onNextCamera() {
    const newCameraIndex = Math.min(
      (this.props.model.selectedRobot?.cameraList.length ?? 0) - 1,
      this.props.model.selectedCameraIndex + 1,
    );
    this.props.controller.onSelectCamera(this.props.model, newCameraIndex);
  }
}

const GridCameraView = observer(
  (props: {
    CameraView: ComponentType<VisionCameraViewProps>;
    camera: VisionCameraModel;
    selectedRobot: VisionRobotModel;
    onClick: () => void;
  }) => {
    return (
      <props.CameraView
        model={props.camera}
        robot={props.selectedRobot}
        viewType="full"
        objectFit="contain"
        onClick={props.onClick}
        allowPanAndZoom={false}
      />
    );
  },
);
