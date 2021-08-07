import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { Mesh } from 'three'
import { Object3D } from 'three'

import { disposableComputed } from '../../../../base/disposable_computed'
import { geometryAndMaterial } from '../../utils'
import { LocalisationRobotModel } from '../model'

import LeftAnkleConfig from './config/left_ankle.json'
import LeftFootConfig from './config/left_foot.json'
import LeftLowerLegConfig from './config/left_lower_leg.json'
import LeftPelvisConfig from './config/left_pelvis.json'
import LeftPelvisYConfig from './config/left_pelvis_y.json'
import LeftUpperLegConfig from './config/left_upper_leg.json'

export class LeftLegViewModel {
  constructor(private model: LocalisationRobotModel) {}

  static of = createTransformer((model: LocalisationRobotModel): LeftLegViewModel => {
    return new LeftLegViewModel(model)
  })

  @computed
  get leftLeg() {
    const leftLeg = new Object3D()
    leftLeg.add(this.leftPelvisY)
    return leftLeg
  }

  @computed
  private get leftPelvisY() {
    const { geometry, materials } = this.leftPelvisYGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0.037, -0.1222, -0.005)
    mesh.rotation.set(0, this.model.motors.leftHipYaw.angle, 0)
    mesh.add(this.leftPelvis)
    return mesh
  }

  @computed
  private get leftPelvis() {
    const { geometry, materials } = this.leftPelvisGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.rotation.set(0, 0, this.model.motors.leftHipRoll.angle)
    mesh.add(this.leftUpperLeg)
    return mesh
  }

  @computed
  private get leftUpperLeg() {
    const { geometry, materials } = this.leftUpperLegGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.rotation.set(this.model.motors.leftHipPitch.angle, 0, 0)
    mesh.add(this.leftLowerLeg)
    return mesh
  }

  @computed
  private get leftLowerLeg() {
    const { geometry, materials } = this.leftLowerLegGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, -0.093, 0)
    mesh.rotation.set(this.model.motors.leftKnee.angle, 0, 0)
    mesh.add(this.leftAnkle)
    return mesh
  }

  @computed
  private get leftAnkle() {
    const { geometry, materials } = this.leftAnkleGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, -0.093, 0)
    mesh.rotation.set(this.model.motors.leftAnklePitch.angle, 0, 0)
    mesh.add(this.leftFoot)
    return mesh
  }

  @computed
  private get leftFoot() {
    const { geometry, materials } = this.leftFootGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.rotation.set(0, 0, this.model.motors.leftAnkleRoll.angle)
    return mesh
  }

  @disposableComputed
  private get leftPelvisYGeometryAndMaterial() {
    return geometryAndMaterial(LeftPelvisYConfig, this.model.color)
  }

  @disposableComputed
  private get leftPelvisGeometryAndMaterial() {
    return geometryAndMaterial(LeftPelvisConfig, this.model.color)
  }

  @disposableComputed
  private get leftUpperLegGeometryAndMaterial() {
    return geometryAndMaterial(LeftUpperLegConfig, this.model.color)
  }

  @disposableComputed
  private get leftLowerLegGeometryAndMaterial() {
    return geometryAndMaterial(LeftLowerLegConfig, this.model.color)
  }

  @disposableComputed
  private get leftAnkleGeometryAndMaterial() {
    return geometryAndMaterial(LeftAnkleConfig, this.model.color)
  }

  @disposableComputed
  private get leftFootGeometryAndMaterial() {
    return geometryAndMaterial(LeftFootConfig, this.model.color)
  }
}
