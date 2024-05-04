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
import style from "./style.module.css";

export type RobotLabelProps = {
  robot: RobotModel;
  selectRobot(robot: RobotModel): void;
};

export const RobotLabel = observer((props: RobotLabelProps) => {
  const { robot, selectRobot } = props;
  const model = RobotLabelModel.of(props.robot);

  const toggleRobot = (robot: RobotModel) => () => selectRobot(robot);
  const toggleStats = action(() => (model.statsOpen = !model.statsOpen));

  const dropdownButtonClassNames = classNames(style.statsDropdownButton, {
    [style.statsDropdownButtonOpen]: model.statsOpen,
  });

  return (
    <>
      <div className={"box-border flex min-w-[256px] hover:bg-gray-200 dark:hover:bg-gray-600"}>
        <label className={"flex items-center flex-grow h-12 px-4 cursor-pointer"}>
          <StatusIndicator className={"mr-3"} connected={robot.connected} />
          <span className={"mr-auto whitespace-nowrap"}>{robot.name}</span>
          <span className={"px-2 mr-1"}>
            <Switch on={robot.enabled} onChange={toggleRobot(robot)} />
          </span>
        </label>
        <button className={dropdownButtonClassNames} onClick={toggleStats}>
          <IconDropdown />
        </button>
      </div>
      <Collapsible open={model.statsOpen} className={style.dataTable}>
        <div className={"box-border width-full text-left"}>
          <div className={"uppercase text-sm text-gray-400 dark:text-gray-300"}>Packets</div>
          <div className={"box-border width-full text-left text-lg"}>{formatSI(model.stats.packets)}</div>
        </div>
        <div className={style.dataCell}>
          <div className={"uppercase text-sm text-gray-400 dark:text-gray-300"}>Packets/s</div>
          <div className={"box-border width-full text-left text-lg"}>{formatSI(model.stats.packetsPerSecond.rate)}</div>
        </div>
        <div className={style.dataCell}>
          <div className={"uppercase text-sm text-gray-400 dark:text-gray-300"}>Bytes</div>
          <div className={"box-border width-full text-left text-lg"}>{formatSI(model.stats.bytes)}</div>
        </div>
        <div className={style.dataCell}>
          <div className={"uppercase text-sm text-gray-400 dark:text-gray-300"}>Bytes/s</div>
          <div className={"box-border width-full text-left text-lg"}>{formatSI(model.stats.bytesPerSecond.rate)}</div>
        </div>
      </Collapsible >
    </>
  );
});
