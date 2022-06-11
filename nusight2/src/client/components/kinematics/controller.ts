import { action } from 'mobx'

import { Vector3 } from '../../math/vector3'
import { Vector2 } from '../../math/vector2'

import { KinematicsModel } from './model'

import { RobotModel } from '../robot/model'
import { KinematicsRobotModel } from './darwin_robot/model'

export class KinematicsController {
  private dragger?: Dragger

  static of(): KinematicsController {
    return new KinematicsController()
  }

  @action
  onAnimationFrame(model: KinematicsModel, time: number) {
    model.time.time = time / 1000
    this.updatePosition(model)
    model.time.lastPhysicsUpdate = time / 1000
  }

  @action
  onWheel(model: KinematicsModel, deltaY: number) {
    const newDistance = model.camera.distance + deltaY / 1000
    model.camera.distance = Math.min(10, Math.max(0.1, newDistance))
  }

  @action
  onMouseDown(model: KinematicsModel, x: number, y: number) {
    const {
      camera: { pitch, yaw },
    } = model
    this.dragger = new Dragger(model, pitch, yaw, Vector2.of(x, y))
  }

  @action
  onMouseMove(x: number, y: number) {
    if (!this.dragger) {
      return
    }
    this.dragger.to = Vector2.of(x, y)
  }

  @action
  onMouseUp(x: number, y: number) {
    if (!this.dragger) {
      return
    }
    this.dragger.to = Vector2.of(x, y)
    this.dragger = undefined
  }

  onSelectRobot(model: KinematicsModel, robot?: RobotModel) {
    model.selectedRobot = robot && KinematicsRobotModel.of(robot)
  }

  private updatePosition(model: KinematicsModel) {
    if (model.robots.length === 0) {
      return
    }

    if (!model.selectedRobot) {
      // TODO: Handle no robots.
      model.selectedRobot = KinematicsRobotModel.of(model.robots[0])
    }

    const target = model.selectedRobot

    const distance = model.camera.distance

    const targetPosition = new Vector3(target.rWTt.x, target.rWTt.y, target.rWTt.z)

    const yaw = -model.controls.yaw
    const pitch = -model.controls.pitch + Math.PI / 2
    const offset = new Vector3(
      Math.sin(pitch) * Math.cos(yaw),
      Math.sin(pitch) * Math.sin(yaw),
      Math.cos(pitch),
    ).multiplyScalar(distance)
    model.camera.position = targetPosition.add(offset)
    model.camera.pitch = pitch - Math.PI / 2
    model.camera.yaw = yaw + Math.PI
  }
}

class Dragger {
  private _to: Vector2

  constructor(
    private readonly model: KinematicsModel,
    private readonly fromPitch: number,
    private readonly fromYaw: number,
    private readonly from: Vector2,
  ) {
    this._to = from
  }

  set to(to: Vector2) {
    this._to = to
    this.update()
  }

  private update() {
    const pitch = Math.max(
      -Math.PI / 2,
      Math.min(Math.PI / 2, this.model.controls.pitch + this._to.y / 200),
    )
    this.model.controls.pitch = pitch
    this.model.controls.yaw = this.model.controls.yaw + this._to.x / 200
  }
}
