import React from 'react'
import { ComponentType } from 'react'
import { ReactNode } from 'react'

import { RobotModel } from '../robot/model'
import { RobotSelector } from '../robot_selector/view'

import style from './style.css'

export function withRobotSelectorMenuBar(
  robots: RobotModel[],
  toggleRobotEnabled: (robot: RobotModel) => void,
) {
  const robotSelector = () => (
    <RobotSelector
      dropdownMenuPosition={'right'}
      robots={robots}
      selectRobot={toggleRobotEnabled}
    />
  )
  return ({ children }: MenuBarProps) => <MenuBar RobotSelector={robotSelector}>{children}</MenuBar>
}

export type MenuBarProps = {
  children?: ReactNode
}

export const MenuBar = ({
  RobotSelector,
  children,
}: MenuBarProps & { RobotSelector: ComponentType }) => (
  <div className={style.menuBar}>
    <div className={style.menu}>{children}</div>
    <div className={style.robotSelector}>
      <RobotSelector />
    </div>
  </div>
)
