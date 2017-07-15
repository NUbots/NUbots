import { action } from 'mobx'
import * as THREE from 'three'
import { Euler } from 'three'
import { Quaternion } from 'three'
import { HIP_TO_FOOT } from './darwin_robot/view_model'
import { KeyCode } from './keycodes'
import { LocalisationModel } from './model'
import { Vector3 } from './model'
import { ViewMode } from './model'

interface KeyModifiers {
  shiftKey: boolean
  ctrlKey: boolean
}

export class LocalisationController {
  public static of(): LocalisationController {
    return new LocalisationController()
  }

  @action
  public onAnimationFrame(model: LocalisationModel, time: number) {
    model.time.time = time / 1000
    this.updatePosition(model)
  }

  @action
  public onLeftClick(model: LocalisationModel, requestPointerLock: () => void) {
    if (!model.locked) {
      requestPointerLock()
    } else {
      model.target = this.getNextTarget(model)
    }
  }

  @action
  public onRightClick(model: LocalisationModel) {
    if (model.locked) {
      model.target = this.getPreviousTarget(model)
    }
  }

  @action
  public onHawkEyeClick(model: LocalisationModel) {
    model.controls.pitch = -Math.PI / 2
    model.controls.yaw = 0
    model.camera.position.set(0, 5, 0)
    model.viewMode = ViewMode.FreeCamera
    this.updatePosition(model)
  }

  @action
  public onPointerLockChange(model: LocalisationModel, locked: boolean): void {
    model.locked = locked
  }

  @action
  public onMouseMove(model: LocalisationModel, movementX: number, movementY: number): void {
    if (!model.locked || model.viewMode === ViewMode.FirstPerson) {
      return
    }

    const pitch = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, model.controls.pitch - movementY / 200))
    model.controls.pitch = pitch
    model.controls.yaw = model.controls.yaw - movementX / 200
  }

  @action
  public onWheel(model: LocalisationModel, deltaY: number): void {
    const newDistance = model.camera.distance + deltaY / 200
    model.camera.distance = Math.min(10, Math.max(0.1, newDistance))
  }

  @action
  public onKeyDown(model: LocalisationModel, key: KeyCode, modifiers: KeyModifiers): void {
    if (model.locked) {
      switch (key) {
        case KeyCode.W:
          model.controls.forward = true
          return
        case KeyCode.A:
          model.controls.left = true
          return
        case KeyCode.S:
          model.controls.back = true
          return
        case KeyCode.D:
          model.controls.right = true
          return
        case KeyCode.R:
          model.controls.up = true
          return
        case KeyCode.F:
          model.controls.down = true
          return
        case KeyCode.Space:
          model.viewMode = this.getNextViewMode(model)

          // TODO (Annable): move this somewhere.
          if (model.viewMode === ViewMode.FreeCamera) {
            model.controls.pitch = model.camera.pitch
            model.controls.yaw = model.camera.yaw
          } else if (model.viewMode === ViewMode.FirstPerson) {
            model.controls.pitch = 0
            model.controls.yaw = 0
          }

          this.updatePosition(model)
          return
        case KeyCode.Enter:
          if (modifiers.shiftKey) {
            model.target = this.getPreviousTarget(model)
          } else {
            model.target = this.getNextTarget(model)
          }
          return
      }
    }
  }

  @action
  public onKeyUp(model: LocalisationModel, key: KeyCode): void {
    if (model.locked) {
      switch (key) {
        case KeyCode.W:
          model.controls.forward = false
          return
        case KeyCode.A:
          model.controls.left = false
          return
        case KeyCode.S:
          model.controls.back = false
          return
        case KeyCode.D:
          model.controls.right = false
          return
        case KeyCode.R:
          model.controls.up = false
          return
        case KeyCode.F:
          model.controls.down = false
          return
      }
    }
  }

  private updatePosition(model: LocalisationModel) {
    switch (model.viewMode) {
      case ViewMode.FreeCamera:
        this.updatePositionNoClip(model)
        return
      case ViewMode.FirstPerson:
        this.updatePositionFirstPerson(model)
        return
      case ViewMode.ThirdPerson:
        this.updatePositionThirdPerson(model)
        return
      default:
        throw new Error(`No view behaviour defined for ${model.viewMode}`)
    }
  }

  private updatePositionNoClip(model: LocalisationModel) {
    const delta = model.time.timeSinceLastRender
    const movement = Vector3.of()
    const movementSpeed = 1
    const actualSpeed = delta * movementSpeed

    if (model.controls.forward) {
      movement.z = movement.z - actualSpeed
    }

    if (model.controls.back) {
      movement.z = movement.z + actualSpeed
    }

    if (model.controls.left) {
      movement.x = movement.x - actualSpeed
    }

    if (model.controls.right) {
      movement.x = movement.x + actualSpeed
    }

    // TODO (Annable): remove THREE dependency from controller.
    const temp = new THREE.Vector3(movement.x, movement.y, movement.z)
    temp.applyEuler(new THREE.Euler(model.controls.pitch, model.controls.yaw, 0, 'YXZ'))
    movement.set(temp.x, temp.y, temp.z)

    // Apply up/down after rotation to keep movement vertical.
    if (model.controls.up) {
      movement.y = movement.y + actualSpeed
    }

    if (model.controls.down) {
      movement.y = movement.y - actualSpeed
    }

    model.camera.pitch = model.controls.pitch
    model.camera.yaw = model.controls.yaw

    model.camera.position.add(movement)
  }

  private updatePositionFirstPerson(model: LocalisationModel) {
    if (model.robots.length === 0) {
      return
    }

    if (!model.target) {
      // TODO (Annable): Handle no robots.
      model.target = model.robots[0]
    }

    const target = model.target

    // This camera position hack will not work with orientation/head movement.
    // TODO (Annable): Sync camera position/rotation properly using kinematic chain.
    model.camera.position.set(target.rWTt.x - 0.001, target.rWTt.y + 0.4, target.rWTt.z)
    const Rwt = new Quaternion(target.Rwt.x, target.Rwt.y, target.Rwt.z, target.Rwt.w)
    const heading = new Euler().setFromQuaternion(Rwt).y
    model.camera.yaw = heading + Math.PI // TODO (Annable): Find why offset by PI is needed.
    model.camera.pitch = 0
  }

  private updatePositionThirdPerson(model: LocalisationModel) {
    if (model.robots.length === 0) {
      return
    }

    if (!model.target) {
      // TODO: Handle no robots.
      model.target = model.robots[0]
    }

    const target = model.target

    const distance = model.camera.distance

    const targetPosition = new Vector3(target.rWTt.x, target.rWTt.y + HIP_TO_FOOT, target.rWTt.z)

    const yaw = model.controls.yaw
    const pitch = -model.controls.pitch + Math.PI / 2
    const offset = new Vector3(
      Math.sin(pitch) * Math.cos(yaw),
      Math.cos(pitch),
      Math.sin(pitch) * Math.sin(yaw),
    ).multiplyScalar(distance)
    const cameraPosition = targetPosition.clone().add(offset)

    model.camera.position.copy(cameraPosition)
    model.camera.pitch = pitch - Math.PI / 2
    model.camera.yaw = -yaw + Math.PI / 2
  }

  private getNextViewMode(model: LocalisationModel) {
    switch (model.viewMode) {
      case ViewMode.FreeCamera:
        return ViewMode.FirstPerson
      case ViewMode.FirstPerson:
        return ViewMode.ThirdPerson
      case ViewMode.ThirdPerson:
        return ViewMode.FreeCamera
      default:
        throw new Error(`No next view mode defined for ${model.viewMode}`)
    }
  }

  private getNextTarget(model: LocalisationModel) {
    // TODO (Annable): ignore robots with visible === false
    const targetIndex = model.robots.findIndex(robot => robot === model.target)
    return model.robots[targetIndex + 1] || model.robots[0]
  }

  private getPreviousTarget(model: LocalisationModel) {
    // TODO (Annable): ignore robots with visible === false
    const targetIndex = model.robots.findIndex(robot => robot === model.target)
    return model.robots[targetIndex - 1] || model.robots[model.robots.length - 1]
  }
}
