import React from "react";
import classNames from "classnames";
import { action } from "mobx";
import { observer } from "mobx-react";

import { Collapsible } from "../../collapsible/view";
import { RobotModel } from "../../robot/model";
import { Switch } from "../../switch/view";
import { StatusIndicator } from "../status_indicator/view";

import { formatSI } from "./format-si";
import IconDropdown from "./icon_dropdown";
import { RobotLabelModel } from "./model";

export type RobotLabelProps = {
  robot: RobotModel;
  selectRobot(robot: RobotModel): void;
};

export const RobotLabel = observer((props: RobotLabelProps) => {
  const { robot, selectRobot } = props;
  const model = RobotLabelModel.of(props.robot);

  const toggleRobot = (robot: RobotModel) => () => selectRobot(robot);
  const toggleStats = action(() => (model.statsOpen = !model.statsOpen));

  const dropdownButtonClassNames = classNames(
    " outline-none cursor-pointer flex-shrink-0 h-[50px] transition-transform duration-300 ease-in-out ",
    {
      ["rotate-180 "]: model.statsOpen,
    },
  );

  return (
    <>
      <div className="flex min-w-[256px] hover:bg-auto-contrast-1 pr-4">
        <label className="flex items-center flex-grow h-12 px-4 cursor-pointer">
          <StatusIndicator className="mr-3" connected={robot.connected} />
          <span className="mr-auto whitespace-nowrap">{robot.name}</span>
          <span className="px-2 mr-1">
            <Switch on={robot.enabled} onChange={toggleRobot(robot)} />
          </span>
        </label>
        <button className={dropdownButtonClassNames} onClick={toggleStats}>
          <IconDropdown />
        </button>
      </div>
      <Collapsible
        open={model.statsOpen}
        className={
          "grid grid-cols-2 grid-rows-2 gap-x-2 gap-y-2 border-0 border-y border-auto bg-auto-surface-1 text-left"
        }
      >
        <div className=" width-full text-left">
          <div className="uppercase text-sm text-gray-600 dark:text-gray-200">Packets</div>
          <div className="width-full text-left text-lg">{formatSI(model.stats.packets)}</div>
        </div>
        <div className="border-box w-full text-left">
          <div className="uppercase text-sm text-gray-600 dark:text-gray-200">Packets/s</div>
          <div className="width-full text-left text-lg">{formatSI(model.stats.packetsPerSecond.rate)}</div>
        </div>
        <div className="border-box w-full text-left">
          <div className="uppercase text-sm text-gray-600 dark:text-gray-200">Bytes</div>
          <div className="width-full text-left text-lg">{formatSI(model.stats.bytes)}</div>
        </div>
        <div className="border-box w-full text-left">
          <div className="uppercase text-sm text-gray-600 dark:text-gray-200">Bytes/s</div>
          <div className="width-full text-left text-lg">{formatSI(model.stats.bytesPerSecond.rate)}</div>
        </div>
      </Collapsible>
    </>
  );
});
