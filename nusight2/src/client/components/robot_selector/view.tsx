import React from "react";
import { observer } from "mobx-react";

import { dropdownContainer } from "../dropdown_container/view";
import { RobotModel } from "../robot/model";

import IconPlug from "./icon_plug";
import IconRobot from "./icon_robot";
import { RobotLabel } from "./robot_label/view";
import style from "./style.module.css";

export type RobotSelectorProps = {
  dropdownMenuPosition?: "left" | "right";
  robots: RobotModel[];
  selectRobot(robot: RobotModel): void;
};

// const svgStyle = {
//   fill: "#F9A50D",
//   height: "2.85em",
//   margin: "0 auto",
//   padding: "0.25em",
//   transition: "fill 0.25s ease-out",
//   width: "auto",
// };

export const RobotSelector = observer((props: RobotSelectorProps) => {
  const { robots, selectRobot } = props;

  const dropdownToggle = (
    <button className="{style.button}">
      {/* <IconRobot style={svgStyle} /> */}
      <IconRobot className="h-9 p-1 w-auto transition-colors duration-75 ease-in-out fill-grey-100 hover:fill-blue-200" />
      Select robots
    </button>
  );
  return (
    <EnhancedDropdown dropdownToggle={dropdownToggle} dropdownPosition={props.dropdownMenuPosition}>
      <div className="bg-white rounded-md shadow-md">
        {robots.length === 0 && (
          <div className="bg-gray-200 p-[1em] text-center">
            <IconPlug className="bg-gray-400 fill-gray-200 h-[3em] w-[3em] rounded-full mx-auto mt-[1em] p-[0.8em]" />
            <div className="text-lg p-1 whitespace-nowrap">No connected robots</div>
            <span className="text-base whitespace-nowrap">Run yarn start:sim to simulate robots</span>
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
