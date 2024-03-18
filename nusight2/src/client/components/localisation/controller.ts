import { action } from "mobx";
import * as THREE from "three";

import { Vector3 } from "../../../shared/math/vector3";

import { KeyCode } from "./keycodes";
import { LocalisationModel } from "./model";
import { ViewMode } from "./model";

interface KeyModifiers {
  shiftKey: boolean;
  ctrlKey: boolean;
}

export class LocalisationController {
  static of(): LocalisationController {
    return new LocalisationController();
  }

  @action
  onAnimationFrame(model: LocalisationModel, time: number) {
    model.time.time = time / 1000;
    this.updatePosition(model);
    model.time.lastPhysicsUpdate = time / 1000;
  }

  @action
  onLeftClick(model: LocalisationModel, requestPointerLock: () => void) {
    if (!model.locked) {
      requestPointerLock();
    } else {
      model.target = this.getNextTarget(model);
    }
  }

  @action
  onRightClick(model: LocalisationModel) {
    if (model.locked) {
      model.target = this.getPreviousTarget(model);
    }
  }

  @action
  onHawkEyeClick(model: LocalisationModel) {
    model.controls.pitch = -Math.PI / 2;
    model.controls.yaw = Math.PI / 2;
    model.camera.position = new Vector3(0, 0, 5);
    model.viewMode = ViewMode.FreeCamera;
    this.updatePosition(model);
  }

  @action
  onPointerLockChange(model: LocalisationModel, locked: boolean): void {
    model.locked = locked;
  }

  @action
  onMouseMove(model: LocalisationModel, movementX: number, movementY: number): void {
    if (!model.locked || model.viewMode === ViewMode.FirstPerson) {
      return;
    }

    const pitch = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, model.controls.pitch - movementY / 200));
    model.controls.pitch = pitch;
    model.controls.yaw = model.controls.yaw - movementX / 200;
  }

  @action
  onWheel(model: LocalisationModel, deltaY: number): void {
    const newDistance = model.camera.distance + deltaY / 1000;
    model.camera.distance = Math.min(10, Math.max(0.1, newDistance));
  }

  @action
  onKeyDown(model: LocalisationModel, key: KeyCode, modifiers: KeyModifiers): void {
    if (model.locked) {
      switch (key) {
        case KeyCode.W:
          model.controls.forward = true;
          return;
        case KeyCode.A:
          model.controls.left = true;
          return;
        case KeyCode.S:
          model.controls.back = true;
          return;
        case KeyCode.D:
          model.controls.right = true;
          return;
        case KeyCode.R:
          model.controls.up = true;
          return;
        case KeyCode.F:
          model.controls.down = true;
          return;
        case KeyCode.Space:
          model.viewMode = this.getNextViewMode(model);

          // TODO (Annable): move this somewhere.
          if (model.viewMode === ViewMode.FreeCamera) {
            model.controls.pitch = model.camera.pitch;
            model.controls.yaw = model.camera.yaw;
          } else if (model.viewMode === ViewMode.FirstPerson) {
            model.controls.pitch = 0;
            model.controls.yaw = 0;
          }

          this.updatePosition(model);
          return;
        case KeyCode.Enter:
          if (modifiers.shiftKey) {
            model.target = this.getPreviousTarget(model);
          } else {
            model.target = this.getNextTarget(model);
          }
          return;
      }
    }
  }

  @action
  onKeyUp(model: LocalisationModel, key: KeyCode): void {
    if (model.locked) {
      switch (key) {
        case KeyCode.W:
          model.controls.forward = false;
          return;
        case KeyCode.A:
          model.controls.left = false;
          return;
        case KeyCode.S:
          model.controls.back = false;
          return;
        case KeyCode.D:
          model.controls.right = false;
          return;
        case KeyCode.R:
          model.controls.up = false;
          return;
        case KeyCode.F:
          model.controls.down = false;
          return;
      }
    }
  }

  private updatePosition(model: LocalisationModel) {
    switch (model.viewMode) {
      case ViewMode.FreeCamera:
        this.updatePositionNoClip(model);
        return;
      case ViewMode.FirstPerson:
        this.updatePositionFirstPerson(model);
        return;
      case ViewMode.ThirdPerson:
        this.updatePositionThirdPerson(model);
        return;
      default:
        throw new Error(`No view behaviour defined for ${model.viewMode}`);
    }
  }

  private updatePositionNoClip(model: LocalisationModel) {
    const delta = model.time.timeSinceLastPhysicsUpdate;
    // TODO (Annable): remove THREE dependency from controller.
    const movement = new THREE.Vector3();
    const movementSpeed = 1;
    const actualSpeed = delta * movementSpeed;

    if (model.controls.forward) {
      movement.x += 1;
    }

    if (model.controls.back) {
      movement.x -= 1;
    }

    if (model.controls.left) {
      movement.y += 1;
    }

    if (model.controls.right) {
      movement.y -= 1;
    }

    movement.applyEuler(new THREE.Euler(0, -model.controls.pitch, model.controls.yaw, "ZXY"));

    // Apply up/down after rotation to keep movement vertical.
    if (model.controls.up) {
      movement.z += 1;
    }

    if (model.controls.down) {
      movement.z -= 1;
    }

    model.camera.pitch = model.controls.pitch;
    model.camera.yaw = model.controls.yaw;

    movement.normalize();
    movement.multiplyScalar(actualSpeed);

    model.camera.position = model.camera.position.add(new Vector3(movement.x, movement.y, movement.z));
  }

  private updatePositionFirstPerson(model: LocalisationModel) {
    if (model.robots.length === 0) {
      return;
    }

    if (!model.target) {
      // TODO (Annable): Handle no robots.
      model.target = model.robots[0];
    }

    const target = model.target;

    // This camera position hack will not work with orientation/head movement.
    // TODO (Annable): Sync camera position/rotation properly using kinematic chain.
    const { translation, rotation } = target.Hft.decompose();
    model.camera.position = translation.add(new Vector3(0, 0, 0.15));
    model.camera.yaw = new THREE.Euler().setFromQuaternion(rotation.toThree()).z;
    model.camera.pitch = 0;
  }

  private updatePositionThirdPerson(model: LocalisationModel) {
    if (model.robots.length === 0) {
      return;
    }

    if (!model.target) {
      // TODO: Handle no robots.
      model.target = model.robots[0];
    }

    const target = model.target;

    const distance = model.camera.distance;

    const targetPosition = target.Hft.decompose().translation;

    const yaw = -model.controls.yaw;
    const pitch = -model.controls.pitch + Math.PI / 2;
    const offset = new Vector3(
      Math.sin(pitch) * Math.cos(yaw),
      Math.sin(pitch) * Math.sin(yaw),
      Math.cos(pitch),
    ).multiplyScalar(distance);
    model.camera.position = targetPosition.add(offset);
    model.camera.pitch = pitch - Math.PI / 2;
    model.camera.yaw = yaw + Math.PI;
  }

  private getNextViewMode(model: LocalisationModel) {
    switch (model.viewMode) {
      case ViewMode.FreeCamera:
        return ViewMode.FirstPerson;
      case ViewMode.FirstPerson:
        return ViewMode.ThirdPerson;
      case ViewMode.ThirdPerson:
        return ViewMode.FreeCamera;
      default:
        throw new Error(`No next view mode defined for ${model.viewMode}`);
    }
  }

  private getNextTarget(model: LocalisationModel) {
    // TODO (Annable): ignore robots with visible === false
    const targetIndex = model.robots.findIndex((robot) => robot === model.target);
    return model.robots[targetIndex + 1] || model.robots[0];
  }

  private getPreviousTarget(model: LocalisationModel) {
    // TODO (Annable): ignore robots with visible === false
    const targetIndex = model.robots.findIndex((robot) => robot === model.target);
    return model.robots[targetIndex - 1] || model.robots[model.robots.length - 1];
  }

  @action
  toggleFieldVisibility = (model: LocalisationModel) => {
    model.fieldVisible = !model.fieldVisible;
  };

  @action
  toggleGridVisibility = (model: LocalisationModel) => {
    model.gridVisible = !model.gridVisible;
  };

  @action
  toggleBallVisibility = (model: LocalisationModel) => {
    model.ballVisible = !model.ballVisible;
  };

  @action
  toggleParticlesVisibility = (model: LocalisationModel) => {
    model.particlesVisible = !model.particlesVisible;
  };

  @action
  toggleRobotVisibility = (model: LocalisationModel) => {
    model.robotVisible = !model.robotVisible;
  };

  @action
  toggleFieldLinePointsVisibility = (model: LocalisationModel) => {
    model.fieldLinePointsVisible = !model.fieldLinePointsVisible;
  };
}
