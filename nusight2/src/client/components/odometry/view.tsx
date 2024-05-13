import { PropsWithChildren } from "react";
import React from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { OdometryController } from "./controller";
import { OdometryModel } from "./model";
import { OdometryVisualizer } from "./odometry_visualizer/view";

@observer
export class OdometryView extends React.Component<{
  controller: OdometryController;
  model: OdometryModel;
  Menu: React.ComponentType<PropsWithChildren>;
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props;
    return (
      <div className={"flex flex-grow flex-shrink flex-col text-center relative w-full h-full bg-gray-200 dark:bg-gray-850"}>
        <Menu>
          <div className={"h-full flex items-center justify-end"}>
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>
        {selectedRobot && (
          <div className={"flex-grow relative"}>
            <OdometryVisualizer model={selectedRobot.visualizerModel} key={selectedRobot.robotModel.id} />
          </div>
        )}
      </div>
    );
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }
}
