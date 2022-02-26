import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { Mesh } from 'three'
import { Object3D } from 'three'

import { disposableComputed } from '../../../../base/disposable_computed'
import { geometryAndMaterial } from '../../utils'
import { LocalisationRobotModel } from '../model'

import RightAnkleConfig from './config/right_ankle.json'
import RightFootConfig from './config/right_foot.json'
import RightLowerLegConfig from './config/right_lower_leg.json'
import RightPelvisConfig from './config/right_pelvis.json'
import RightPelvisYConfig from './config/right_pelvis_y.json'
import RightUpperLegConfig from './config/right_upper_leg.json'

export class RightLegViewModel {
  constructor(private model: LocalisationRobotModel) {}

  static of = createTransformer((model: LocalisationRobotModel): RightLegViewModel => {
    return new RightLegViewModel(model)
  })

  @computed
  get rightLeg() {
    const rightLeg = new Object3D()
    rightLeg.add(this.rightPelvisY)
    return rightLeg
  }

  @computed
  private get rightPelvisY() {
    const { geometry, materials } = this.rightPelvisYGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(-0.037, -0.1222, -0.005)
    mesh.rotation.set(0, this.model.motors.rightHipYaw.angle, 0)
    mesh.add(this.rightPelvis)
    return mesh
  }

  @computed
  private get rightPelvis() {
    const { geometry, materials } = this.rightPelvisGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.rotation.set(0, 0, this.model.motors.rightHipRoll.angle)
    mesh.add(this.rightUpperLeg)
    return mesh
  }

  @computed
  private get rightUpperLeg() {
    const { geometry, materials } = this.rightUpperLegGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.rotation.set(this.model.motors.rightHipPitch.angle, 0, 0)
    mesh.add(this.rightLowerLeg)
    return mesh
  }

  @computed
  private get rightLowerLeg() {
    const { geometry, materials } = this.rightLowerLegGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, -0.093, 0)
    mesh.rotation.set(this.model.motors.rightKnee.angle, 0, 0)
    mesh.add(this.rightAnkle)
    return mesh
  }

  @computed
  private get rightAnkle() {
    const { geometry, materials } = this.rightAnkleGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, -0.093, 0)
    mesh.rotation.set(this.model.motors.rightAnklePitch.angle, 0, 0)
    mesh.add(this.rightFoot)
    return mesh
  }

  @computed
  private get rightFoot() {
    const { geometry, materials } = this.rightFootGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.rotation.set(0, 0, this.model.motors.rightAnkleRoll.angle)
    return mesh
  }

  @disposableComputed
  private get rightPelvisYGeometryAndMaterial() {
    return geometryAndMaterial(RightPelvisYConfig, this.model.color)
  }

  @disposableComputed
  private get rightPelvisGeometryAndMaterial() {
    return geometryAndMaterial(RightPelvisConfig, this.model.color)
  }

  @disposableComputed
  private get rightUpperLegGeometryAndMaterial() {
    return geometryAndMaterial(RightUpperLegConfig, this.model.color)
  }

  @disposableComputed
  private get rightLowerLegGeometryAndMaterial() {
    return geometryAndMaterial(RightLowerLegConfig, this.model.color)
  }

  @disposableComputed
  private get rightAnkleGeometryAndMaterial() {
    return geometryAndMaterial(RightAnkleConfig, this.model.color)
  }

  @disposableComputed
  private get rightFootGeometryAndMaterial() {
    return geometryAndMaterial(RightFootConfig, this.model.color)
  }
}
