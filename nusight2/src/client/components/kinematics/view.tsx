import React, { PropsWithChildren } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { KinematicsController } from "./controller";
import { KinematicsModel } from "./model";
import { KinematicsRobotModel } from "./robot_model";
import { CanvasWrapper } from "./r3f_components/canvas_wrapper";

const JointDataDisplay: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  return (
    <div className="p-4 border border-black dark:border-white rounded-lg w-full">
      <h3 className="text-xl font-semibold mb-4 pb-2">Joint Angles</h3>
      <ul className="space-y-2">
        {Object.entries(robot.motors).map(([jointName, motor]) => {
          // Format joint name from camelCase to "Camel Case"
          const formattedLabel = jointName
            .replace(/([a-z])([A-Z])/g, "$1 $2")
            .replace(/^./, (match) => match.toUpperCase());
          return (
            <li
              key={jointName}
              className="flex justify-between items-center p-2 border-black dark:border-white border-b"
            >
              <span className="font-medium">{formattedLabel}</span>
              <span className="text-right">{motor.angle.toFixed(2)}°</span>
            </li>
          );
        })}
      </ul>
    </div>
  );
});

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
