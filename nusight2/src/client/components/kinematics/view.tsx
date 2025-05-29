import React, { PropsWithChildren, useState } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { KinematicsController } from "./controller";
import { KinematicsModel } from "./model";
import { CanvasWrapper } from "./r3f_components/canvas_wrapper";
import { KinematicsRobotModel } from "./robot_model";

const JointDataDisplay: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  const [unit, setUnit] = useState<"rad" | "deg">("rad");

  return (
    <div className="p-4 border border-black dark:border-white rounded-lg w-full">
      <div className="flex justify-between items-center mb-4 pb-2">
        <h3 className="text-xl font-semibold">Joint Angles</h3>
        <button
          onClick={() => setUnit(unit === "rad" ? "deg" : "rad")}
          className="text-sm px-2 py-1 border rounded hover:bg-gray-200 dark:hover:bg-gray-700"
        >
          Show in {unit === "rad" ? "Degrees" : "Radians"}
        </button>
      </div>
      <ul className="divide-y divide-black dark:divide-white">
        {Object.entries(robot.motors).map(([jointName, motor]) => {
          const formattedLabel = jointName
            .replace(/([a-z])([A-Z])/g, "$1 $2")
            .replace(/^./, (match) => match.toUpperCase());

          const angle =
            unit === "rad" ? `${motor.angle.toFixed(2)} rad` : `${((motor.angle * 180) / Math.PI).toFixed(2)}°`;

          return (
            <li key={jointName} className="grid grid-cols-[auto_auto] gap-4 p-2">
              <span className="font-medium">{formattedLabel}</span>
              <span className="justify-self-end">{angle}</span>
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
            <div className="flex-1 relative">
              <CanvasWrapper selectedRobot={selectedRobot} />
            </div>

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
