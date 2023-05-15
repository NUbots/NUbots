import { computed } from "mobx";
import { action } from "mobx";
import { observer } from "mobx-react";
import { PropsWithChildren } from "react";
import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";
import { CameraModel } from "./camera/model";
import { CameraViewProps } from "./camera/view";
import { VisionController } from "./controller";
import { GridLayout } from "./grid_layout/grid_layout";

import { VisionModel } from "./model";
import styles from "./style.module.css";

@observer
export class VisionView extends Component<{
  controller: VisionController;
  model: VisionModel;
  Menu: ComponentType<PropsWithChildren>;
  CameraView: ComponentType<CameraViewProps>;
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
      CameraView,
    } = this.props;
    return (
      <div className={styles.vision}>
        <Menu>
          <div className={styles.selector}>
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>
        {selectedRobot && (
          <div className={styles.content}>
            <GridLayout itemAspectRatio={this.itemAspectRatio}>
              {selectedRobot?.cameraList.map((camera) => (
                <CameraView key={camera.id} model={camera} />
              ))}
            </GridLayout>
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
    const firstCamera: CameraModel | undefined = selectedRobot?.cameras.values().next().value;
    return firstCamera ? firstCamera.image.width / firstCamera.image.height : 1;
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }
}
