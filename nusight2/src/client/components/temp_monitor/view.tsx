import React, { PropsWithChildren } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { TempMonitorController } from "./controller";
import { ServoNames, TempMonitorModel } from "./model";

const tempWarningThreshold = 50;

@observer
export class TempMonitorView extends React.Component<{
  controller: TempMonitorController;
  model: TempMonitorModel;
  Menu: React.ComponentType<PropsWithChildren>;
}> {
  prevHighestTemp: number = 0;
  tempAlert = (highestTemp: number, servoId: string) => {
    if (highestTemp > tempWarningThreshold && highestTemp > this.prevHighestTemp) {
      alert(`Temperature of ${servoId} is too high!`);
    }
    this.prevHighestTemp = highestTemp;
  };

  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
    } = this.props;

    if (selectedRobot?.highestTemperatureServo) {
      const { temperature, name } = selectedRobot.highestTemperatureServo;
      this.tempAlert(temperature, name);
    }
    return (
      <div className="flex flex-grow flex-shrink flex-col text-center relative w-full h-full">
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
          <div className="overflow-x-auto p-8 border-t border-auto">
            <h2 className="text-xl font-bold mb-4">Servo Temperatures</h2>
            <div className="grid grid-cols-4 gap-4">
              {Array.from(selectedRobot.servoTemperatures.entries()).map(([id, temp]) => (
                <div
                  key={id}
                  className={
                    "p-4 rounded-lg border border-collapse " +
                    (selectedRobot.highestTemperatureServo &&
                    selectedRobot.highestTemperatureServo.temperature > tempWarningThreshold
                      ? "bg-red-700 border-red-900 dark:border-red-500"
                      : "bg-auto-surface-1 border-auto")
                  }
                >
                  <h3 className="font-semibold">{ServoNames[id] || `Servo ${id}`}</h3>
                  <p>ID: {id}</p>
                  <p className={`text-lg ${temp > tempWarningThreshold ? "text-white" : "text-green-600"}`}>{temp}°C</p>
                </div>
              ))}
            </div>
            <div
              className="mt-8 grid grid-cols-2 gap-+
            4"
            >
              {selectedRobot.highestTemperatureServo && (
                <>
                  <div
                    className={
                      "text-white p-4 rounded-lg border border-collapse " +
                      (selectedRobot.highestTemperatureServo.temperature > tempWarningThreshold
                        ? "bg-red-700 border-red-900 dark:border-red-500"
                        : "bg-auto-surface-1 border-auto")
                    }
                  >
                    <h3 className="text-lg font-semibold ">Highest Temperature</h3>
                    <p className="text-2xl font-bold">
                      {selectedRobot.highestTemperatureServo.temperature.toFixed(1)}°C
                    </p>
                    <p className="">
                      {selectedRobot.highestTemperatureServo.name} (ID: {selectedRobot.highestTemperatureServo.id})
                    </p>
                  </div>
                </>
              )}
              <div className="bg-auto-surface-1 p-4 rounded-lg  border border-collapse border-auto">
                <h3 className="text-lg font-semibold">Average Temperature</h3>
                <p className="text-2xl font-bold">{selectedRobot.averageTemperature.toFixed(1)}°C</p>
              </div>
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
