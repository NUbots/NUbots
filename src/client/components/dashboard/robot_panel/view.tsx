import * as classNames from 'classnames'
import * as React from 'react'
import BallIcon from './icon/ball.svg'
import BatteryIcon from './icon/battery.svg'
import CameraIcon from './icon/camera.svg'
import GoalIcon from './icon/goal.svg'
import WarningIcon from './icon/warning.svg'
import * as style from './style.css'
import { Vector3 } from '../../../math/vector3'

export type LastStatus = 'okay' | 'warning' | 'danger'

export type RobotPanelProps = {
  batteryValue?: string
  behaviour: string
  lastCameraImage: LastStatus
  lastSeenBall: LastStatus
  lastSeenGoal: LastStatus
  mode: string
  penalised: boolean
  penalty: string
  phase: string
  title: string
  walkCommand: Vector3
}

export const RobotPanel = (props: RobotPanelProps) => {
  const cameraClassName = classNames(style.icon, style.cameraIcon, {
    [style.iconWarningStatus]: props.lastCameraImage === 'warning',
    [style.iconDangerStatus]: props.lastCameraImage === 'danger',
  })
  const ballClassName = classNames(style.icon, style.ballIcon, {
    [style.iconWarningStatus]: props.lastSeenBall === 'warning',
    [style.iconDangerStatus]: props.lastSeenBall === 'danger',
  })
  const goalClassName = classNames(style.icon, style.goalIcon, {
    [style.iconWarningStatus]: props.lastSeenGoal === 'warning',
    [style.iconDangerStatus]: props.lastSeenGoal === 'danger',
  })
  return (
    <div>
      <header className={style.header}>
        <div className={style.statusBar}>
          <span className={style.title}>
            {props.title}
          </span>
          {props.batteryValue && <Battery value={props.batteryValue}/>}
        </div>
      </header>
      <div className={style.details}>
        <div className={style.group}>
          <div className={style.row}>
            <span className={style.label}>Mode</span>
            {props.mode}
          </div>
          <div className={style.row}>
            <span className={style.label}>Phase</span>
            {props.phase}
          </div>
          <div className={style.row}>
            <span className={style.label}>Behaviour</span>
            {props.behaviour}
          </div>
          <div className={style.row}>
            <span className={style.label}>Penalty</span>
            <div className={style.value}>
              <span className={style.penalty}>
                {props.penalty}
              </span>
              {props.penalised && <WarningIcon className={style.penaltyIcon}/>}
            </div>
          </div>
          <div className={style.row}>
            <span className={style.label}>Walk Command</span>
            {props.walkCommand.x.toFixed(3)}, {props.walkCommand.y.toFixed(3)}, {props.walkCommand.z.toFixed(3)}
          </div>
        </div>
        <div className={style.icons}>
          <span className={cameraClassName}>
            <CameraIcon/>
          </span>
          <span className={ballClassName}>
            <BallIcon/>
          </span>
          <span className={goalClassName}>
            <GoalIcon/>
          </span>
        </div>
      </div>
    </div>
  )
}

const Battery = (props: { value: string }) => (
  <span className={style.battery}>
    <span className={style.batteryValue}>
      {props.value}
    </span>
    <BatteryIcon className={style.batteryIcon}/>
  </span>
)

