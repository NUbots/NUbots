import React from "react";
import classNames from "classnames";
import { action } from "mobx";
import { observer } from "mobx-react";

import { Collapsible } from "../../collapsible/view";
import { Icon } from "../../icon/view";
import { RobotModel } from "../../robot/model";
import { Switch } from "../../switch/view";
import { StatusIndicator } from "../status_indicator/view";

import { formatSI } from "./format-si";
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
      <div className={style.robot}>
        <label className={style.robotLabel}>
          <StatusIndicator className={style.robotStatusIndicator} connected={robot.connected} />
          <span className={style.robotName}>{robot.name}</span>
          <span className={style.robotSwitch}>
            <Switch on={robot.enabled} onChange={toggleRobot(robot)} />
          </span>
        </label>
        <button className={dropdownButtonClassNames} onClick={toggleStats}>
          <Icon weight={400}>expand_more</Icon>
        </button>
      </div>
      <Collapsible open={model.statsOpen} className={style.dataTable}>
        <div className={style.dataCell}>
          <div className={style.dataLabel}>Packets</div>
          <div className={style.dataValue}>{formatSI(model.stats.packets)}</div>
        </div>
        <div className={style.dataCell}>
          <div className={style.dataLabel}>Packets/s</div>
          <div className={style.dataValue}>{formatSI(model.stats.packetsPerSecond.rate)}</div>
        </div>
        <div className={style.dataCell}>
          <div className={style.dataLabel}>Bytes</div>
          <div className={style.dataValue}>{formatSI(model.stats.bytes)}</div>
        </div>
        <div className={style.dataCell}>
          <div className={style.dataLabel}>Bytes/s</div>
          <div className={style.dataValue}>{formatSI(model.stats.bytesPerSecond.rate)}</div>
        </div>
      </Collapsible>
    </>
  );
});
