import React, { PropsWithChildren } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";
import { KinematicsController } from "./controller";
import { KinematicsModel } from "./model";
import { CanvasWrapper } from "./r3f_components/canvas_wrapper/view";
import { JointDataDisplay } from "./r3f_components/joint_data_display/view";
@observer
export class KinematicsView extends React.Component<{
  controller: KinematicsController;
  model: KinematicsModel;
  Menu: React.ComponentType<PropsWithChildren>;
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props;

    return (
      <div className="w-full h-screen flex flex-col">
        <Menu>
          <div className="h-full flex items-center justify-end">
            <RobotSelectorSingle
              autoSelect={true}
              robots={robots}
              selected={selectedRobot?.robotModel}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>

        {selectedRobot && (
          <div className="flex flex-1 overflow-hidden">
            {/* Left Side: 3D Canvas */}
            <div className="flex-1 relative">
              <CanvasWrapper selectedRobot={selectedRobot} />
            </div>

            {/* Right Side: Joint Data Pane */}
            <div className="w-1/4 h-full overflow-y-auto">
              <JointDataDisplay robot={selectedRobot} />
            </div>
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
