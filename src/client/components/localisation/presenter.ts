import { action } from 'mobx'
import * as THREE from 'three'
import { HIP_TO_FOOT } from './darwin_robot/view_model'
import { KeyCode } from './keycodes'
import { LocalisationModel } from './model'
import { Vector3 } from './model'
import { ViewMode } from './model'
import { LocalisationView } from './view'

interface KeyModifiers {
  shiftKey: boolean
  ctrlKey: boolean
}

export class LocalisationPresenter {
  private model: LocalisationModel

  constructor(opts: { model: LocalisationModel }) {
    Object.assign(this, opts)
  }

  public static of(opts: { model: LocalisationModel }) {
    return new LocalisationPresenter(opts)
  }

  @action
  public onAnimationFrame(time: number) {
    this.model.time.time = time / 1000
    this.updatePosition()
  }

  @action
  public onLeftClick(view: LocalisationView) {
    if (!this.model.locked) {
      view.requestPointerLock()
    } else {
      this.model.target = this.getNextTarget()
    }
  }

  @action
  public onRightClick() {
    if (this.model.locked) {
      this.model.target = this.getPreviousTarget()
    }
  }

  @action
  public onHawkEyeClick() {
    this.model.controls.pitch = -Math.PI / 2
    this.model.controls.yaw = 0
    this.model.camera.position.set(0, 5, 0)
    this.model.viewMode = ViewMode.NO_CLIP
    this.updatePosition()
  }

  @action
  public onPointerLockChange(locked: boolean): void {
    this.model.locked = locked
  }

  @action
  public onMouseMove(movementX: number, movementY: number): void {
    if (!this.model.locked || this.model.viewMode === ViewMode.FIRST_PERSON) {
      return
    }

    const pitch = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, this.model.controls.pitch - movementY / 200))
    this.model.controls.pitch = pitch
    this.model.controls.yaw = this.model.controls.yaw - movementX / 200
  }

  @action
  public onWheel(deltaY: number): void {
    const newDistance = this.model.camera.distance + deltaY / 200
    this.model.camera.distance = Math.min(10, Math.max(0.1, newDistance))
  }

  @action
  public onKeyDown(key: KeyCode, modifiers: KeyModifiers): void {
    if (this.model.locked) {
      switch (key) {
        case KeyCode.W:
          this.model.controls.forward = true
          return
        case KeyCode.A:
          this.model.controls.left = true
          return
        case KeyCode.S:
          this.model.controls.back = true
          return
        case KeyCode.D:
          this.model.controls.right = true
          return
        case KeyCode.R:
          this.model.controls.up = true
          return
        case KeyCode.F:
          this.model.controls.down = true
          return
        case KeyCode.Space:
          this.model.viewMode = this.getNextViewMode()

          // TODO (Annable): move this somewhere.
          if (this.model.viewMode === ViewMode.NO_CLIP) {
            this.model.controls.pitch = this.model.camera.pitch
            this.model.controls.yaw = this.model.camera.yaw
          } else if (this.model.viewMode === ViewMode.FIRST_PERSON) {
            this.model.controls.pitch = 0
            this.model.controls.yaw = 0
          }

          this.updatePosition()
          return
        case KeyCode.Enter:
          if (modifiers.shiftKey) {
            this.model.target = this.getPreviousTarget()
          } else {
            this.model.target = this.getNextTarget()
          }
          return
      }
    }
  }

  @action
  public onKeyUp(key: KeyCode): void {
    if (this.model.locked) {
      switch (key) {
        case KeyCode.W:
          this.model.controls.forward = false
          return
        case KeyCode.A:
          this.model.controls.left = false
          return
        case KeyCode.S:
          this.model.controls.back = false
          return
        case KeyCode.D:
          this.model.controls.right = false
          return
        case KeyCode.R:
          this.model.controls.up = false
          return
        case KeyCode.F:
          this.model.controls.down = false
          return
      }
    }
  }

  private updatePosition() {
    switch (this.model.viewMode) {
      case ViewMode.NO_CLIP:
        this.updatePositionNoClip()
        return
      case ViewMode.FIRST_PERSON:
        this.updatePositionFirstPerson()
        return
      case ViewMode.THIRD_PERSON:
        this.updatePositionThirdPerson()
        return
      default:
        throw new Error(`No view behaviour defined for ${this.model.viewMode}`)
    }
  }

  private updatePositionNoClip() {
    const delta = this.model.time.timeSinceLastRender
    const movement = Vector3.of()
    const movementSpeed = 1
    const actualSpeed = delta * movementSpeed

    if (this.model.controls.forward) {
      movement.z = movement.z - actualSpeed
    }

    if (this.model.controls.back) {
      movement.z = movement.z + actualSpeed
    }

    if (this.model.controls.left) {
      movement.x = movement.x - actualSpeed
    }

    if (this.model.controls.right) {
      movement.x = movement.x + actualSpeed
    }

    // TODO (Annable): remove THREE dependency from presenter.
    const temp = new THREE.Vector3(movement.x, movement.y, movement.z)
    temp.applyEuler(new THREE.Euler(this.model.controls.pitch, this.model.controls.yaw, 0, 'YXZ'))
    movement.set(temp.x, temp.y, temp.z)

    // Apply up/down after rotation to keep movement vertical.
    if (this.model.controls.up) {
      movement.y = movement.y + actualSpeed
    }

    if (this.model.controls.down) {
      movement.y = movement.y - actualSpeed
    }

    this.model.camera.pitch = this.model.controls.pitch
    this.model.camera.yaw = this.model.controls.yaw

    this.model.camera.position.add(movement)
  }

  private updatePositionFirstPerson() {
    if (this.model.robots.length === 0) {
      return
    }

    if (!this.model.target) {
      // TODO (Annable): Handle no robots.
      this.model.target = this.model.robots[0]
    }

    const target = this.model.target

    // This camera position hack will not work with orientation/head movement.
    // TODO (Annable): Sync camera position/rotation properly using kinematic chain.
    this.model.camera.position.set(target.position.x - 0.001, target.position.y + 0.4, target.position.z)
    this.model.camera.yaw = target.heading + Math.PI // TODO (Annable): Find why offset by PI is needed.
    this.model.camera.pitch = 0
  }

  private updatePositionThirdPerson() {
    if (this.model.robots.length === 0) {
      return
    }

    if (!this.model.target) {
      // TODO: Handle no robots.
      this.model.target = this.model.robots[0]
    }

    const target = this.model.target

    const distance = this.model.camera.distance

    const targetPosition = new Vector3(target.position.x, target.position.y + HIP_TO_FOOT, target.position.z)

    const yaw = this.model.controls.yaw
    const pitch = -this.model.controls.pitch + Math.PI / 2
    const offset = new Vector3(
        Math.sin(pitch) * Math.cos(yaw),
        Math.cos(pitch),
        Math.sin(pitch) * Math.sin(yaw),
    ).multiplyScalar(distance)
    const cameraPosition = targetPosition.clone().add(offset)

    this.model.camera.position.copy(cameraPosition)
    this.model.camera.pitch = pitch - Math.PI / 2
    this.model.camera.yaw = -yaw + Math.PI / 2
  }

  private getNextViewMode() {
    switch (this.model.viewMode) {
      case ViewMode.NO_CLIP:
        return ViewMode.FIRST_PERSON
      case ViewMode.FIRST_PERSON:
        return ViewMode.THIRD_PERSON
      case ViewMode.THIRD_PERSON:
        return ViewMode.NO_CLIP
      default:
        throw new Error(`No next view mode defined for ${this.model.viewMode}`)
    }
  }

  private getNextTarget() {
    const targetIndex = this.model.robots.findIndex(robot => robot === this.model.target)
    return this.model.robots[targetIndex + 1] || this.model.robots[0]
  }

  private getPreviousTarget() {
    const targetIndex = this.model.robots.findIndex(robot => robot === this.model.target)
    return this.model.robots[targetIndex - 1] || this.model.robots[this.model.robots.length - 1]
  }
}
