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
      <div className="flex flex-col w-full">
        <Menu>
          <ul className="list-none m-0 p-0 h-full flex items-stretch">
            <li className="flex m-0 p-0">
              {/* <button className={style.menuButton} onClick={this.onToggleOrientationClick}></button> */}
              <button className="bg-transparent text-black p-4 text-sm border-2 border-transparent cursor-pointer transition-colors duration-75 ease-in-out hover:border-gray-300" onClick={this.onToggleOrientationClick}>
                Flip Orientation
              </button>
            </li>
          </ul>
        </Menu>
        {/* Difference between flex + flex-1 and flex-1 alone?*/}
        <div className="flex flex-1 flex-col">
          <div className="flex-1 relative">
            <Field />
          </div>
          {showPanels && (
            <div className="flex p-3.5">
              {model.robots.map((robot) => {
                const model = RobotPanelViewModel.of(robot);
                return (
                  robot.enabled && (
                    <div className="flex-1 mx-1 rounded-lg shadow-md" key={robot.id}>
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
                        title={model.title}
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
