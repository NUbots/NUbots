import { observer } from 'mobx-react'
import React from 'react'

import { dropdownContainer } from '../dropdown_container/view'
import { RobotModel } from '../robot/model'

import PlugIcon from './plug.svg'
import RobotIcon from './robot.svg'
import { RobotLabel } from './robot_label/view'
import style from './style.css'

export type RobotSelectorProps = {
  dropdownMenuPosition?: 'left' | 'right'
  robots: RobotModel[]
  selectRobot(robot: RobotModel): void
}

export const RobotSelector = observer((props: RobotSelectorProps) => {
  const { robots, selectRobot } = props
  const dropdownToggle = (
    <button className={style.button}>
      <RobotIcon />
      Select robots
    </button>
  )
  return (
    <div className={style.robotSelector}>
      <EnhancedDropdown
        dropdownToggle={dropdownToggle}
        dropdownPosition={props.dropdownMenuPosition}
      >
        <div className={style.robots}>
          {robots.length === 0 && (
            <div className={style.empty}>
              <div className={style.emptyIcon}>
                <PlugIcon />
              </div>
              <div className={style.emptyTitle}>No connected robots</div>
              <span className={style.emptyDescription}>Run yarn start:sim to simulate robots</span>
            </div>
          )}
          {robots.map(robot => (
            <RobotLabel key={robot.id} robot={robot} selectRobot={selectRobot} />
          ))}
        </div>
      </EnhancedDropdown>
    </div>
  )
})

const EnhancedDropdown = dropdownContainer()
