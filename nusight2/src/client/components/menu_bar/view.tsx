import React from "react";
import { ComponentType } from "react";
import { ReactNode } from "react";

import { AppModel } from "../app/model";
import { withNbsScrubbers } from "../nbs_scrubbers/view";
import { RobotModel } from "../robot/model";
import { RobotSelector } from "../robot_selector/view";

export function withRobotSelectorMenuBar(appModel: AppModel, toggleRobotEnabled: (robot: RobotModel) => void) {
  const robotSelector = () => (
    <RobotSelector dropdownMenuPosition={"right"} robots={appModel.robots} selectRobot={toggleRobotEnabled} />
  );

  const { NbsScrubbers, NbsScrubbersToggle } = withNbsScrubbers(appModel);

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
    <div className="flex flex-col bg-auto-surface-1">
      <div className="flex h-[60px]">
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
