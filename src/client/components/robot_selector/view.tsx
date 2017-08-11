import { observer } from 'mobx-react'
import * as React from 'react'
import { dropdownContainer } from '../dropdown_container/view'
import { Switch } from '../switch/view'
import { RobotModel } from '../robot/model'
import PlugIcon from './plug.svg'
import RobotIcon from './robot.svg'
import * as style from './style.css'

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
  const onChange = (robot: RobotModel) => () => selectRobot(robot)
  const dropdownMenuClassName = props.dropdownMenuPosition === 'right' ? style.rightDropdownMenu : ''
  return (
    <div className={style.robotSelector}>
      <EnhancedDropdown dropdownToggle={dropdownToggle} dropdownMenuClassName={dropdownMenuClassName}>
        <div className={style.robots}>
          {robots.length === 0 &&
            <div className={style.empty}>
              <div className={style.emptyIcon}>
                <PlugIcon />
              </div>
              <div className={style.emptyTitle}>
                No connected robots
              </div>
              <span className={style.emptyDescription}>
                Run yarn start:sim to simulate robots
              </span>
            </div>
          }
          {robots.map(robot => {
            return (
              <label key={robot.id} className={style.robot}>
                <span className={style.robotLabel}>{robot.name}</span>
                <Switch on={robot.enabled} onChange={onChange(robot)} />
              </label>
            )
          })}
        </div>
      </EnhancedDropdown>
    </div>
  )
})

const EnhancedDropdown = dropdownContainer()
