import React from "react";
import classNames from "classnames";

import { Vector3 } from "../../../../shared/math/vector3";
import { Icon } from "../../icon/view";

import GoalIcon from "./icon/goal";


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
  const connectionStatusClassName = classNames({
    "w-3 h-3 rounded-full border border-green-400 mr-2": true,
    "bg-green-200": props.connected,
    "bg-red": !props.connected,
  });
  const cameraClassName = classNames({
    "items-center flex justify-around": true,
    ["text-orange-400"]: props.lastCameraImage === "warning",
    ["text-red"]: props.lastCameraImage === "danger",
  });
  const ballClassName = classNames({
    "items-center flex justify-around": true,
    ["text-orange-400"]: props.lastSeenBall === "warning",
    ["text-red"]: props.lastSeenBall === "danger",
  });
  const goalClassName = classNames({
    "items-center flex justify-around h-6 w-6": true,
    ["fill-orange-400"]: props.lastSeenGoal === "warning",
    ["fill-red"]: props.lastSeenGoal === "danger",
  });
  return (
    <div>
      <header className="bg-gray-700 dark:bg-gray-800 text-white rounded-t-md">
        <div className="items-center flex text-sm h-7 px-2">
          <span className={connectionStatusClassName} title={props.connected ? "Connected" : "Disconnected"} />
          <span className="flex-1">{props.title}</span>
          {props.batteryValue && <Battery value={props.batteryValue} />}
        </div>
      </header>
      <div className="text-sm rounded-b-md p-2 bg-gray-100 dark:bg-gray-850">
        <div className="mb-2 pb-3">
          <div className="flex flex-col">
            <span className="flex-1 font-bold">Mode</span>
            {props.mode}
          </div>
          <div className="flex flex-col">
            <span className="flex-1 font-bold">Phase</span>
            {props.phase}
          </div>
          <div className="flex flex-col">
            <span className="flex-1 font-bold">Penalty</span>
            <div className="items-center flex">
              <span>{props.penalty}</span>
              {props.penalised && (
                <Icon className="ml-1.5 text-warning" fill>
                  warning
                </Icon>
              )}
            </div>
          </div>
          <div className="flex flex-col">
            <span className="flex-1 font-bold">Walk Command</span>
            {props.walkCommand.x.toFixed(3)}, {props.walkCommand.y.toFixed(3)}, {props.walkCommand.z.toFixed(3)}
          </div>
        </div>
        <div className="items-center flex justify-around fill-black dark:fill-white">
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
    </div >
  );
};

const Battery = (props: { value: string }) => (
  <span className="items-center flex">
    <span className="mr-1">{props.value}</span>
    <Icon className="h-6">battery_full</Icon>
  </span>
);
