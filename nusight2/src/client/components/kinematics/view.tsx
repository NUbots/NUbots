import React, { PropsWithChildren, useState } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { KinematicsController } from "./controller";
import { KinematicsModel, ServoNames } from "./model";
import { CanvasWrapper } from "./r3f_components/canvas_wrapper";
import { KinematicsRobotModel } from "./robot_model";

const ServoDataDisplay: React.FC<{ robot: KinematicsRobotModel }> = observer(({ robot }) => {
  const [unit, setUnit] = useState<"rad" | "deg">("rad");

  return (
    <div className="p-4 border border-black dark:border-white rounded-lg w-full">
      <div className="flex justify-between items-center mb-4 pb-2">
        <h3 className="text-xl font-semibold">Servo Information</h3>
        <button
          onClick={() => setUnit(unit === "rad" ? "deg" : "rad")}
          className="text-sm px-2 py-1 border rounded hover:bg-gray-300 dark:hover:bg-gray-600 border-black dark:border-white"
        >
          Show in {unit === "rad" ? "Degrees" : "Radians"}
        </button>
      </div>

      {/* Temperature Summary */}
      <div className="mb-4 grid grid-cols-2 gap-4">
        <div className="bg-gray-100 dark:bg-gray-800 p-3 rounded-lg">
          <h4 className="text-sm font-medium text-gray-600 dark:text-gray-400">Highest Temperature</h4>
          {robot.highestTemperatureServo && (
            <div className="text-lg font-bold">
              <span className={robot.highestTemperatureServo.temperature > 50 ? "text-red-600" : "text-[#888888]"}>
                {robot.highestTemperatureServo.temperature.toFixed(1)}°C
              </span>
              <div className="text-xs text-gray-500">{robot.highestTemperatureServo.name}</div>
            </div>
          )}
        </div>
        <div className="bg-gray-100 dark:bg-gray-800 p-3 rounded-lg">
          <h4 className="text-sm font-medium text-gray-600 dark:text-gray-400">Average Temperature</h4>
          <div className="text-lg font-bold text-[#888888]">{robot.averageTemperature.toFixed(1)}°C</div>
        </div>
      </div>

      {/* Servo Table */}
      <div className="overflow-x-auto">
        <table className="w-full text-sm">
          <thead>
            <tr className="border-b border-gray-300 dark:border-gray-600">
              <th className="text-left p-2 font-medium">Servo</th>
              <th className="text-right p-2 font-medium">Angle</th>
              <th className="text-right p-2 font-medium">Temperature</th>
            </tr>
          </thead>
          <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
            {Object.entries(robot.motors).map(([jointName, motor], index) => {
              const servoId = index;
              const servoName = ServoNames[servoId] || jointName.replace(/([a-z])([A-Z])/g, "$1 $2").replace(/^./, (match) => match.toUpperCase());
              const temperature = robot.servoTemperatures.get(servoId);
              const angle = unit === "rad" ? `${motor.angle.toFixed(2)} rad` : `${((motor.angle * 180) / Math.PI).toFixed(2)}°`;
              const isOverLimit = temperature !== undefined && temperature > 50;

              return (
                <tr
                  key={jointName}
                  className={`hover:bg-gray-50 dark:hover:bg-gray-800 ${isOverLimit ? 'bg-red-50 dark:bg-red-900/20' : ''
                    }`}
                >
                  <td className="p-2 font-medium">{servoName}</td>
                  <td className="p-2 text-right">{angle}</td>
                  <td className="p-2 text-right">
                    {temperature !== undefined ? (
                      <span className={isOverLimit ? "text-red-600 font-medium" : "text-[#888888]"}>
                        {temperature.toFixed(1)}°C
                      </span>
                    ) : (
                      <span className="text-gray-400">-</span>
                    )}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
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

            <div className="w-1/3 h-full overflow-y-auto">
              <ServoDataDisplay robot={selectedRobot} />
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
