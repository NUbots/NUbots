import React from "react";
import classNames from "classnames";

import { Vector3 } from "../../../../shared/math/vector3";
import { Icon } from "../../icon/view";

import GoalIcon from "./icon/goal";
import style from "./style.module.css";

export type LastStatus = "okay" | "warning" | "danger";

export type RobotPanelProps = {
  connected: boolean;
  batteryValue?: string;
  lastCameraImage: LastStatus;
  lastSeenBall: LastStatus;
  lastSeenGoal: LastStatus;
  mode: string;
  penalised: boolean;
  penalty: string;
  phase: string;
  title: string;
  walkCommand: Vector3;
};

export const RobotPanel = (props: RobotPanelProps) => {
  const connectionStatusClassName = classNames(style.connectionStatus, {
    [style.connectedStatus]: props.connected,
    [style.disconnectedStatus]: !props.connected,
  });
  const cameraClassName = classNames(style.icon, style.cameraIcon, {
    [style.iconWarningStatus]: props.lastCameraImage === "warning",
    [style.iconDangerStatus]: props.lastCameraImage === "danger",
  });
  const ballClassName = classNames(style.icon, style.ballIcon, {
    [style.iconWarningStatus]: props.lastSeenBall === "warning",
    [style.iconDangerStatus]: props.lastSeenBall === "danger",
  });
  const goalClassName = classNames(style.icon, style.goalIcon, {
    [style.iconWarningStatus]: props.lastSeenGoal === "warning",
    [style.iconDangerStatus]: props.lastSeenGoal === "danger",
  });
  return (
    <div>
      <header className={style.header}>
        <div className={style.statusBar}>
          <span className={connectionStatusClassName} title={props.connected ? "Connected" : "Disconnected"} />
          <span className={style.title}>{props.title}</span>
          {props.batteryValue && <Battery value={props.batteryValue} />}
        </div>
      </header>
      <div className={style.details}>
        <div className={style.group}>
          <div className={style.row}>
            <span className={style.label}>Mode</span>
            {props.mode}
          </div>
          <div className={style.row}>
            <span className={style.label}>Phase</span>
            {props.phase}
          </div>
          <div className={style.row}>
            <span className={style.label}>Penalty</span>
            <div className={style.value}>
              <span className={style.penalty}>{props.penalty}</span>
              {props.penalised && (
                <Icon className={style.penaltyIcon} fill>
                  warning
                </Icon>
              )}
            </div>
          </div>
          <div className={style.row}>
            <span className={style.label}>Walk Command</span>
            {props.walkCommand.x.toFixed(3)}, {props.walkCommand.y.toFixed(3)}, {props.walkCommand.z.toFixed(3)}
          </div>
        </div>
        <div className={style.icons}>
          <span className={cameraClassName}>
            <Icon>photo_camera</Icon>
          </span>
          <span className={ballClassName}>
            <Icon>sports_soccer</Icon>
          </span>
          <span className={goalClassName}>
            <GoalIcon />
          </span>
        </div>
      </div>
    </div>
  );
};

const Battery = (props: { value: string }) => (
  <span className={style.battery}>
    <span className={style.batteryValue}>{props.value}</span>
    <Icon className={style.batteryIcon}>battery_full</Icon>
  </span>
);
