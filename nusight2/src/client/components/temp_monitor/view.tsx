import React, { PropsWithChildren } from "react";
import { action } from "mobx";
import { observer } from "mobx-react";
import { Icon } from "../icon/view";
import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";
import { TempMonitorController } from "./controller";
import { TempMonitorModel, TempMonitorRobotModel, ServoNames } from "./model";

@observer
export class TempMonitorView extends React.Component<{
  controller: TempMonitorController;
  model: TempMonitorModel;
  Menu: React.ComponentType<PropsWithChildren>;
}> {
  render() {
    const {
      model: { selectedRobot, robots },
      Menu,
      controller,
    } = this.props;

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
                <div key={id} className="bg-auto-surface-1 p-4 rounded-lg  border border-collapse border-auto">
                  <h3 className="font-semibold">{ServoNames[id] || `Servo ${id}`}</h3>
                  <p>ID: {id}</p>
                  <p className={`text-lg ${temp > 50 ? "text-red-600" : "text-green-600"}`}>{temp.toFixed(1)}°C</p>
                </div>
              ))}
            </div>
            <div className="mt-8 grid grid-cols-2 gap-4">
              <div className="bg-auto-surface-1 p-4 rounded-lg  border border-collapse border-auto">
                <h3 className="text-lg font-semibold ">Highest Temperature</h3>
                {selectedRobot.highestTemperatureServo && (
                  <>
                    <p className="text-2xl font-bold">
                      {selectedRobot.highestTemperatureServo.temperature.toFixed(1)}°C
                    </p>
                    <p className="">
                      {selectedRobot.highestTemperatureServo.name} (ID: {selectedRobot.highestTemperatureServo.id})
                    </p>
                  </>
                )}
              </div>
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
