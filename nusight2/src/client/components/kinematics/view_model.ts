import { computed } from 'mobx'
import * as THREE from 'three'
import { HemisphereLight, Object3D, PointLight, Scene } from 'three'
import { Vector3 } from '../../math/vector3'
import { group, perspectiveCamera, scene, stage } from '../three/builders'
import { Canvas } from '../three/three'
import { KinematicsRobotModel } from './darwin_robot/model'
import { KinematicsModel } from './model'
import { NUgusViewModel } from './nugus_robot/view_model'

export class KinematicsViewModel {
  private readonly canvas: Canvas
  private readonly model: KinematicsModel

  constructor(canvas: Canvas, model: KinematicsModel) {
    this.canvas = canvas
    this.model = model
  }

  static of(canvas: Canvas, model: KinematicsModel) {
    return new KinematicsViewModel(canvas, model)
  }

  readonly stage = stage(() => ({ camera: this.camera(), scene: this.getScene() }))

  private readonly camera = perspectiveCamera(() => ({
    fov: 75,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.01,
    far: 100,
    position: this.model.camera.position,
    rotation: Vector3.from({
      x: Math.PI / 2 + this.model.camera.pitch,
      y: 0,
      z: -Math.PI / 2 + this.model.camera.yaw,
    }),
    rotationOrder: 'ZXY',
    up: Vector3.from({ x: 0, y: 0, z: 1 }),
  }))

  getScene(): Scene {
    const scene = this.scene()
    scene.background = new THREE.Color('#ffffff')
    return scene
  }

  private readonly scene = scene(() => ({
    children: [this?.robot, this.hemisphereLight, this.pointLight],
  }))

  @computed
  private get robot(): Object3D {
    let newModel
    if (this.model.selectedRobot) {
      newModel = NUgusViewModel.of(this.model.selectedRobot)
    } else {
      newModel = NUgusViewModel.of(KinematicsRobotModel.of(this.model.robots[0]))
    }
    const newRobot = newModel.robot
    for (const joint of Object.keys(this.model.enabledJoints)) {
      const object = newRobot.getObjectByName(`${joint}_kinematics`)
      if (this.model.enabledJoints[joint]) {
        if (!object) newRobot.getObjectByName(joint)?.add(this.jointLines(joint))
      } else {
        if (object) newRobot.getObjectByName(joint)?.remove(object)
      }
    }
    return newRobot
  }

  @computed
  private get hemisphereLight(): HemisphereLight {
    return new HemisphereLight('#fff', '#fff', 0.6)
  }

  private readonly jointLines = (mesh: string): Object3D => {
    const lines = group(() => ({
      children: [createLine([0, 0, 0.1], 0x0000ff, 3), createLine([0, 0.1, 0], 0xff0000, 3)],
    }))
    const lineGroup = <Object3D>lines()
    lineGroup.name = `${mesh}_kinematics`
    return lineGroup
  }

  private readonly worldLines = group(() => ({
    children: [
      createLine([0, 0, 10], 0x0000ff),
      createLine([0, 10, 0], 0xff0000),
      createLine([10, 0, 0], 0x00ff00),
    ],
  }))

  @computed
  private get pointLight() {
    const light = new PointLight('#fff')
    light.position.set(
      this.model.camera.position.x,
      this.model.camera.position.y,
      this.model.camera.position.z,
    )
    return light
  }
}

const createLine = (
  to: [number, number, number],
  color: any = '0xffffff',
  linewidth: number = 1,
  from: [number, number, number] = [0, 0, 0],
): THREE.Line => {
  return new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(...from),
      new THREE.Vector3(...to),
    ]),
    new THREE.LineBasicMaterial({ color, linewidth }),
  )
}
