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
      <IconRobot className="h-9 p-1 w-auto transition-colors duration-75 ease-in-out fill-grey-100 hover:fill-orange-100" />
      Select robots
    </button>
  );
  return (
    <div className="{style.robotSelector}">
      <EnhancedDropdown dropdownToggle={dropdownToggle} dropdownPosition={props.dropdownMenuPosition}>
        <div className="bg-white rounded-md shadow-md">
          {robots.length === 0 && (
            <div className="rounded-md bg-gray-200 p-3 text-center w-auto mx-auto">
              <div className="bg-gray-800 h-11 w-11 rounded-full mx-auto">
                <IconPlug className="fill-gray-200 h-6 w-6" />
              </div>
              <div className="text-lg p-1 whitespace-nowrap">No connected robots</div>
              <span className="text-base whitespace-nowrap">Run yarn start:sim to simulate robots</span>
            </div>
          )}
          {robots.map((robot) => (
            <RobotLabel key={robot.id} robot={robot} selectRobot={selectRobot} />
          ))}
        </div>
      </EnhancedDropdown>
    </div>
  );
});

const EnhancedDropdown = dropdownContainer();
