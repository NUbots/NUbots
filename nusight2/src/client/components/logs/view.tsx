import React, { ComponentType, PropsWithChildren } from "react";
import { observer } from "mobx-react";

import { LogsController } from "./controller";
import { LogsModel } from "./model";
import { LogsNetwork } from "./network";
import { Icon } from "../icon/view";
import { RobotSelectorSingle } from "../robot_selector_single/view";

export interface LogsViewProps {
  controller: LogsController;
  Menu: ComponentType<PropsWithChildren>;
  model: LogsModel;
  network: LogsNetwork;
}

export const LogsView = observer(function LogsView(props: LogsViewProps) {
  const { Menu, model, controller } = props;

  const { robots, selectedLogsRobot } = model;

  return (
    <div className="w-full flex flex-col">
      <Menu>
        <div className="h-full flex items-center justify-end">
          <RobotSelectorSingle
            autoSelect
            robots={robots}
            selected={selectedLogsRobot?.robotModel}
            onSelect={controller.onSelectRobot}
          />
        </div>
      </Menu>
      {selectedLogsRobot ? (
        <div className="flex-grow border-t border-gray-300 flex flex-col">
          <div className="relative h-full w-full">
            Show the logs for "{selectedLogsRobot.robotModel.name}" robot here!
          </div>
        </div>
      ) : (
        <div className="flex flex-col justify-center items-center h-full w-full bg-gray-100">
          <div className="text-icon mb-2">
            <Icon size={48}>electrical_services</Icon>
          </div>
          <div className="text-2xl">No connected robots</div>
          <p className="text-sm opacity-80 mt-2">Connect a robot or load an NBS file to view logs</p>
        </div>
      )}
    </div>
  );
});
