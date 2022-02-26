import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { Mesh } from 'three'
import { Object3D } from 'three'

import { disposableComputed } from '../../../../base/disposable_computed'
import { geometryAndMaterial } from '../../utils'
import { LocalisationRobotModel } from '../model'

import LeftLowerArmConfig from './config/left_lower_arm.json'
import LeftShoulderConfig from './config/left_shoulder.json'
import LeftUpperArmConfig from './config/left_upper_arm.json'

export class LeftArmViewModel {
  constructor(private model: LocalisationRobotModel) {}

  static of = createTransformer((model: LocalisationRobotModel): LeftArmViewModel => {
    return new LeftArmViewModel(model)
  })

  @computed
  get leftArm() {
    const leftArm = new Object3D()
    leftArm.add(this.leftShoulder)
    return leftArm
  }

  @computed
  private get leftShoulder() {
    const { geometry, materials } = this.leftShoulderGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0.082, 0, 0)
    mesh.rotation.set(this.model.motors.leftShoulderPitch.angle - Math.PI / 2, 0, 0)
    mesh.add(this.leftUpperArm)
    return mesh
  }

  @computed
  private get leftUpperArm() {
    const { geometry, materials } = this.leftUpperArmGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, -0.016, 0)
    mesh.rotation.set(0, 0, this.model.motors.leftShoulderRoll.angle)
    mesh.add(this.leftLowerArm)
    return mesh
  }

  @computed
  private get leftLowerArm() {
    const { geometry, materials } = this.leftLowerArmGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, -0.06, 0.016)
    mesh.rotation.set(this.model.motors.leftElbow.angle, 0, 0)
    return mesh
  }

  @disposableComputed
  private get leftShoulderGeometryAndMaterial() {
    return geometryAndMaterial(LeftShoulderConfig, this.model.color)
  }

  @disposableComputed
  private get leftUpperArmGeometryAndMaterial() {
    return geometryAndMaterial(LeftUpperArmConfig, this.model.color)
  }

  @disposableComputed
  private get leftLowerArmGeometryAndMaterial() {
    return geometryAndMaterial(LeftLowerArmConfig, this.model.color)
  }
}
