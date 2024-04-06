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
      <IconRobot className="h-9 p-1 w-auto transition duration-75 ease-in-out fill-grey-100 hover:fill-orange-100" />
      Select robots
    </button>
  );
  return (
    <div className="{style.robotSelector}">
      <EnhancedDropdown dropdownToggle={dropdownToggle} dropdownPosition={props.dropdownMenuPosition}>
        <div className="{style.robots}">
          {robots.length === 0 && (
            <div className="{style.empty}">
              <div className="{style.emptyIcon}">
                <IconPlug />
              </div>
              <div className="{style.emptyTitle}">No connected robots</div>
              <span className="{style.emptyDescription}">Run yarn start:sim to simulate robots</span>
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
