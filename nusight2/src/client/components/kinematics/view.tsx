import { reaction } from 'mobx'
import { computed } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { observer } from 'mobx-react'
import { now } from 'mobx-utils'
import React from 'react'

import { Canvas } from '../three/three'
import { Three } from '../three/three'

import { KinematicsController } from './controller'
import { KinematicsModel } from './model'
import style from './style.css'
import { KinematicsViewModel } from './view_model'
import { RobotSelectorSingle } from '../robot_selector_single/view'
import { RobotModel } from '../robot/model'

type KinematicsViewProps = {
  controller: KinematicsController
  Menu: React.ComponentType
  model: KinematicsModel
}

@observer
export class KinematicsView extends React.Component<KinematicsViewProps> {
  private readonly canvas = React.createRef<Three>()

  componentDidMount(): void {
    document.addEventListener('mousemove', this.onMouseMove, false)
    document.addEventListener('mousedown', this.onMouseDown, false)
    document.addEventListener('mouseup', this.onMouseUp, false)
    document.addEventListener('wheel', this.onWheel, false)
    disposeOnUnmount(
      this,
      reaction(() => now('frame'), this.onAnimationFrame),
    )
  }

  componentWillUnmount(): void {
    document.removeEventListener('mousemove', this.onMouseMove, false)
    document.addEventListener('mousedown', this.onMouseDown, false)
    document.addEventListener('mouseup', this.onMouseUp, false)
    document.removeEventListener('wheel', this.onWheel, false)
  }

  render(): JSX.Element {
    console.log(this.props.model.enabledJoints)
    const {
      model: { selectedRobot, robots, enabledJoints },
      Menu,
    } = this.props
    return (
      <div className={style.kinematics}>
        <Menu>
          <div className={style.selector}>
            <RobotSelectorSingle
              robots={robots}
              selected={selectedRobot?.model}
              onSelect={this.onSelectRobot}
            />
          </div>
        </Menu>
        <div className={style.kinematics__interactive}>
          <div className={style.kinematics__canvas}>
            <Three ref={this.canvas} stage={this.stage} />
          </div>
          <div>
            {Object.keys(enabledJoints).map(joint => (
              <p
                key={joint}
                style={{ color: enabledJoints[joint] ? 'green' : 'red' }}
                onClick={() => this.setMeshes(joint)}
              >
                {joint}
              </p>
            ))}
          </div>
        </div>
        `
      </div>
    )
  }

  private stage = (canvas: Canvas) => computed(() => [this.viewModel(canvas).stage])

  private viewModel = (canvas: Canvas) => KinematicsViewModel.of(canvas, this.props.model)

  requestPointerLock() {
    this.canvas.current!.requestPointerLock()
  }

  private onAnimationFrame = (time: number) => {
    this.props.controller.onAnimationFrame(this.props.model, time)
  }

  private onMouseDown = (e: MouseEvent) => {
    this.props.controller.onMouseDown(this.props.model, e.movementX, e.movementY)
  }

  private onMouseUp = (e: MouseEvent) => {
    this.props.controller.onMouseUp(e.movementX, e.movementY)
  }

  private onMouseMove = (e: MouseEvent) => {
    this.props.controller.onMouseMove(e.movementX, e.movementY)
  }

  private onWheel = (e: WheelEvent) => {
    e.preventDefault()
    this.props.controller.onWheel(this.props.model, e.deltaY)
  }

  private onSelectRobot = (robot?: RobotModel) => {
    this.props.controller.onSelectRobot(this.props.model, robot)
  }

  private setMeshes = (joint: string) => {
    this.props.model.setMeshes(joint)
  }
}
