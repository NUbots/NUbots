import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { Mesh } from 'three'

import { disposableComputed } from '../../../../base/disposable_computed'
import { geometryAndMaterial } from '../../utils'
import { HeadViewModel } from '../head/view_model'
import { LeftArmViewModel } from '../left_arm/view_model'
import { LeftLegViewModel } from '../left_leg/view_model'
import { LocalisationRobotModel } from '../model'
import { RightArmViewModel } from '../right_arm/view_model'
import { RightLegViewModel } from '../right_leg/view_model'

import BodyConfig from './config/body.json'

export class BodyViewModel {
  constructor(private model: LocalisationRobotModel) {}

  static of = createTransformer((model: LocalisationRobotModel): BodyViewModel => {
    return new BodyViewModel(model)
  })

  @computed
  get body(): Mesh {
    const { geometry, materials } = this.bodyGeometryAndMaterial
    const mesh = new Mesh(geometry, materials)
    mesh.position.set(0, 0, 0.096)
    mesh.add(this.head)
    mesh.add(this.leftArm)
    mesh.add(this.rightArm)
    mesh.add(this.leftLeg)
    mesh.add(this.rightLeg)
    mesh.rotation.x = Math.PI / 2
    mesh.rotation.y = Math.PI / 2
    return mesh
  }

  @disposableComputed
  private get bodyGeometryAndMaterial() {
    return geometryAndMaterial(BodyConfig, this.model.color)
  }

  @computed
  private get head() {
    return HeadViewModel.of(this.model).head
  }

  @computed
  private get leftArm() {
    return LeftArmViewModel.of(this.model).leftArm
  }

  @computed
  private get rightArm() {
    return RightArmViewModel.of(this.model).rightArm
  }

  @computed
  private get leftLeg() {
    return LeftLegViewModel.of(this.model).leftLeg
  }

  @computed
  private get rightLeg() {
    return RightLegViewModel.of(this.model).rightLeg
  }
}
