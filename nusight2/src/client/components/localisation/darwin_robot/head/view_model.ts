import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { Mesh } from 'three'
import { Object3D } from 'three'

import { disposableComputed } from '../../../../base/disposable_computed'
import { geometryAndMaterial } from '../../utils'
import { LocalisationRobotModel } from '../model'

import CameraConfig from './config/camera.json'
import EyeLEDConfig from './config/eye_led.json'
import HeadConfig from './config/head.json'
import HeadLEDConfig from './config/head_led.json'
import NeckConfig from './config/neck.json'

export class HeadViewModel {
  constructor(private model: LocalisationRobotModel) {}

  static of = createTransformer((model: LocalisationRobotModel): HeadViewModel => {
    return new HeadViewModel(model)
  })

  @computed
  get head() {
    const head = new Object3D()
    head.add(this.neck)
    return head
  }

  @computed
  private get neck() {
    const { geometry, materials } = this.neckGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, 0.051, 0)
    mesh.rotation.set(0, this.model.motors.headPan.angle, 0)
    mesh.add(this.skull)
    return mesh
  }

  @computed
  private get skull() {
    const { geometry, materials } = this.skullGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.rotation.set(this.model.motors.headTilt.angle, 0, 0)
    mesh.add(this.headLED)
    mesh.add(this.eyeLED)
    mesh.add(this.camera)
    return mesh
  }

  @computed
  private get headLED() {
    const { geometry, materials } = this.headLEDGeometryAndMaterial
    return new Mesh(geometry, materials)
  }

  @computed
  private get eyeLED() {
    const { geometry, materials } = this.eyeLEDGeometryAndMaterial
    return new Mesh(geometry, materials)
  }

  @computed
  private get camera() {
    const { geometry, materials } = this.cameraGeometryAndMaterial
    return new Mesh(geometry, materials)
  }

  @disposableComputed
  private get neckGeometryAndMaterial() {
    return geometryAndMaterial(NeckConfig, this.model.color)
  }

  @disposableComputed
  private get skullGeometryAndMaterial() {
    return geometryAndMaterial(HeadConfig, this.model.color)
  }

  @disposableComputed
  private get headLEDGeometryAndMaterial() {
    return geometryAndMaterial(HeadLEDConfig, this.model.color)
  }

  @disposableComputed
  private get eyeLEDGeometryAndMaterial() {
    return geometryAndMaterial(EyeLEDConfig, this.model.color)
  }

  @disposableComputed
  private get cameraGeometryAndMaterial() {
    return geometryAndMaterial(CameraConfig, this.model.color)
  }
}
