import * as React from 'react'
import { ComponentType } from 'react'
import { ReactNode } from 'react'
import { StatelessComponent } from 'react'
import { RobotModel } from '../robot/model'
import { RobotSelector } from '../robot_selector/view'
import * as style from './style.css'

export function withRobotSelectorMenuBar(robots: RobotModel[], toggleRobotEnabled: (robot: RobotModel) => void) {
  const robotSelector = () => {
    return (
      <RobotSelector dropdownMenuPosition={'right'} robots={robots} selectRobot={toggleRobotEnabled} />
    )
  }
  return (props: MenuBarProps) => (
    <MenuBar robotSelector={robotSelector}>{props.children}</MenuBar>
  )
}

export type MenuBarProps = {
  children?: ReactNode
  robotSelector: ComponentType<{}>
}

export const MenuBar: StatelessComponent<MenuBarProps> = (props: MenuBarProps) => {
  const RobotSelector = props.robotSelector
  return (
    <div className={style.menuBar}>
      <div className={style.menu}>
        {props.children}
      </div>
      <div className={style.robotSelector}>
        <RobotSelector />
      </div>
    </div>
  )
}
