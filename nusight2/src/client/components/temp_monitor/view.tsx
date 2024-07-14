import { PropsWithChildren } from "react";
import React from "react";
import { action } from "mobx";
import { observer } from "mobx-react";

import { Icon } from "../icon/view";
import { RobotModel } from "../robot/model";
import { RobotSelectorSingle } from "../robot_selector_single/view";

import { TempMonitorController } from "./controller";
import { TempMonitorModel, TempMonitorRobotModel } from "./model";

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
        {selectedRobot && <div className="overflow-x-auto p-8 border-t border-auto">Hello world</div>}
      </div>
    );
  }

  @action.bound
  private onSelectRobot(robot?: RobotModel) {
    this.props.controller.onSelectRobot(this.props.model, robot);
  }
}
