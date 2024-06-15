import { PropsWithChildren } from "react";
import React from "react";
import { Component } from "react";
import { ComponentType } from "react";
import { observer } from "mobx-react";

import { Button } from "../button/button";

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
      <div className={"flex flex-col w-full"}>
        <Menu>
          <ul className="list-none h-full flex items-center">
            <li className="flex px-4">
              <Button onClick={this.onToggleOrientationClick}>Flip Orientation</Button>
            </li>
          </ul>
        </Menu>
        <div className="flex flex-col flex-1 bg-auto-surface-0">
          <div className="flex-1 relative">
            <Field />
          </div>
          {showPanels && (
            <div className="flex p-2">
              {model.robots.map((robot) => {
                const model = RobotPanelViewModel.of(robot);
                return (
                  robot.enabled && (
                    <div className="rounded-sm shadow-md flex-1 ml-2 overflow-hidden first:ml-0" key={robot.id}>
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
