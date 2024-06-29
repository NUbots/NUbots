import React from "react";
import { observer } from "mobx-react";

import { dropdownContainer } from "../dropdown_container/view";
import { RobotModel } from "../robot/model";

import IconPlug from "./icon_plug";
import IconRobot from "./icon_robot";
import { RobotLabel } from "./robot_label/view";

export type RobotSelectorProps = {
  dropdownMenuPosition?: "left" | "right";
  robots: RobotModel[];
  selectRobot(robot: RobotModel): void;
};

export const RobotSelector = observer((props: RobotSelectorProps) => {
  const { robots, selectRobot } = props;

  const dropdownToggle = (
    <button className="items-center justify-center cursor-pointer flex flex-col fill-auto-primary">
      <IconRobot className="h-9 p-1 w-auto transition-colors duration-75 ease-in-out" />
      Select robots
    </button>
  );
  return (
    <EnhancedDropdown dropdownToggle={dropdownToggle} dropdownPosition={props.dropdownMenuPosition}>
      <div className="bg-auto-surface-2 shadow-md text-auto-primary">
        {robots.length === 0 && (
          <div className="p-[1em] text-center">
            <IconPlug className="bg-nusight-500 h-[3em] w-[3em] rounded-full mx-auto mt-[1em] p-[0.8em]" />
            <div className="text-lg p-1 whitespace-nowrap">No connected robots</div>
            <span className="text-base whitespace-nowrap text-nusight-500">Run yarn start:sim to simulate robots</span>
          </div>
        )}
        {robots.map((robot) => (
          <RobotLabel key={robot.id} robot={robot} selectRobot={selectRobot} />
        ))}
      </div>
    </EnhancedDropdown>
  );
});

const EnhancedDropdown = dropdownContainer();
