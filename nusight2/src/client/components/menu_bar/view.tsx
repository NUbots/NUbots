import React from "react";
import { ComponentType } from "react";
import { ReactNode } from "react";

import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";
import { RobotSelector } from "../robot_selector/view";

import { withNbsScrubbers } from "./nbs_scrubbers/view";

export function withRobotSelectorMenuBar(
  robots: RobotModel[],
  toggleRobotEnabled: (robot: RobotModel) => void,
  nusightNetwork: NUsightNetwork,
) {
  const robotSelector = () => (
    <RobotSelector dropdownMenuPosition={"right"} robots={robots} selectRobot={toggleRobotEnabled} />
  );

  const { NbsScrubbers, NbsScrubbersToggle } = withNbsScrubbers(nusightNetwork);

  return ({ children }: MenuBarProps) => (
    <MenuBar RobotSelector={robotSelector} NbsScrubbers={NbsScrubbers} NbsScrubbersToggle={NbsScrubbersToggle}>
      {children}
    </MenuBar>
  );
}

export type MenuBarProps = {
  children?: ReactNode;
};

export const MenuBar = ({
  RobotSelector,
  NbsScrubbers,
  NbsScrubbersToggle,
  children,
}: MenuBarProps & {
  RobotSelector: ComponentType;
  NbsScrubbers: ComponentType;
  NbsScrubbersToggle: ComponentType;
}) => {
  return (
    <div className="flex flex-col bg-gray-200 dark:bg-gray-850 text-gray-900 dark:text-gray-100">
      <div className="flex h-[60px] space-x-10 my-2 mx-2">
        <div className="flex-1">{children}</div>
        <div>
          <RobotSelector />
        </div>
        <div>
          <NbsScrubbersToggle />
        </div>
      </div>
      <NbsScrubbers />
    </div>
  );
};
