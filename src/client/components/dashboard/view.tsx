import { observer } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'
import { ComponentType } from 'react'
import { DashboardModel } from './model'
import { DashboardNetwork } from './network'
import { RobotPanel } from './robot_panel/view'
import { RobotPanelViewModel } from './robot_panel/view_model'
import * as style from './style.css'

export type DashboardProps = {
  Field: ComponentType<{}>
  menu: ComponentType<{}>
  model: DashboardModel
  network: DashboardNetwork
}

@observer
export class Dashboard extends Component<DashboardProps> {
  public componentWillUnmount(): void {
    this.props.network.destroy()
  }

  public render() {
    const { menu: Menu, model } = this.props
    const showPanels = model.robots.some(robot => robot.visible)
    const Field = this.props.Field
    return (
      <div className={style.page}>
        <Menu/>
        <div className={style.dashboard}>
          <div className={style.field}>
            <Field/>
          </div>
          {showPanels &&
          <div className={style.panels}>
            {model.robots.map(robot => {
              const model = RobotPanelViewModel.of(robot)
              return (
                robot.visible &&
                <div className={style.panel} key={robot.name}>
                  <RobotPanel
                    batteryValue={model.batteryValue}
                    behaviour={model.behaviour}
                    lastCameraImage={model.lastCameraImage}
                    lastSeenBall={model.lastSeenBall}
                    lastSeenGoal={model.lastSeenGoal}
                    mode={model.mode}
                    penalised={model.penalised}
                    penalty={model.penalty}
                    phase={model.phase}
                    title={model.title}/>
                </div>
              )
            })}
          </div>
          }
        </div>
      </div>
    )
  }
}
