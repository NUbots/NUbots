import { PropsWithChildren } from "react";
import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import { observer } from "mobx-react";

import { DashboardController } from "./controller";
import { DashboardModel } from "./model";
import { DashboardNetwork } from "./network";
import { RobotPanel } from "./robot_panel/view";
import { RobotPanelViewModel } from "./robot_panel/view_model";

export type DashboardProps = {
  controller: DashboardController;
  Field: ComponentType;
  Menu: ComponentType<PropsWithChildren>;
  model: DashboardModel;
  network: DashboardNetwork;
};

@observer
export class Dashboard extends Component<DashboardProps> {
  componentWillUnmount(): void {
    this.props.network.destroy();
  }

  render() {
    const { Menu, model } = this.props;
    const showPanels = model.robots.some((robot) => robot.enabled);
    const Field = this.props.Field;
    return (
      <div className="flex flex-col w-full bg-gray-100 dark:bg-gray-700 dark:text-white">
        <div className="bg-gray-200 dark:bg-gray-600">
          <Menu>
            <ul className="list-none h-full flex items-stretch">
              <li className="flex">
                <button className="bg-transparent font-bold p-4 text-md" onClick={this.onToggleOrientationClick}>
                  Flip Orientation
                </button>
              </li>
            </ul>
          </Menu>
        </div>
        <div className="flex flex-1 flex-col dark:bg-gray-700">
          <div className="flex-1 relative m-5">
            <Field />
          </div>
          {showPanels && (
            <div className="flex m-4">
              {model.robots.map((robot) => {
                const model = RobotPanelViewModel.of(robot);
                return (
                  robot.enabled && (
                    <div className="flex-1 mx-2 rounded-lg shadow-md bg-gray-200 dark:bg-gray-600 " key={robot.id}>
                      <RobotPanel
                        connected={model.connected}
                        batteryValue={model.batteryValue}
                        lastCameraImage={model.lastCameraImage}
                        lastSeenBall={model.lastSeenBall}
                        lastSeenGoal={model.lastSeenGoal}
                        mode={model.mode}
                        penalised={model.penalised}
                        penalty={model.penalty}
                        phase={model.phase}
                        title={"model.title"}
                        walkCommand={model.walkCommand}
                      />
                    </div>
                  )
                );
              })}
            </div>
          )}
        </div>
      </div>
    );
  }

  private onToggleOrientationClick = () => {
    const { controller, model } = this.props;
    controller.toggleOrientation(model);
  };
}
