import { computed } from 'mobx'
import '@react-three/fiber'
import { observer } from 'mobx-react'
import React from 'react'
import { Color } from 'three'
import { Mesh } from 'three'
import { Object3D } from 'three'

import { Vector3 } from '../../../shared/math/vector3'
import { meshBasicMaterial } from '../three/builders'
import { circleBufferGeometry } from '../three/builders'
import { scene } from '../three/builders'
import { PerspectiveCamera } from '../three/three_fiber'

import { FieldView } from './field/view'
import { LocalisationModel } from './model'
import { NUgusViewModel } from './nugus_robot/view_model'
import { SkyboxViewModel } from './skybox/view_model'

@observer
export class LocalisationViewModel extends React.Component<{
  model: LocalisationModel
}> {
  render() {
    return <object3D>
      <PerspectiveCamera
        args={[75, 1, 0.01, 100]}
        position={this.model.camera.position.toArray()}
        rotation={[Math.PI / 2 + this.model.camera.pitch, 0, -Math.PI / 2 + this.model.camera.yaw, 'ZXY']}
        up={[0, 0, 1]}
      >
        <pointLight color="white"/>
      </PerspectiveCamera>
      <FieldView model={this.model.field}/>
      <SkyboxViewModel model={this.model.skybox}/>
      <hemisphereLight args={['#fff', '#fff', 0.6]}/>
    </object3D>
  }

  private get model() {
    return this.props.model
  }

  private readonly scene = scene(() => ({
    children: [...this.robots, this.fieldLineDots],
  }))

  @computed
  private get robots(): Object3D[] {
    return this.model.robots
      .filter((robotModel) => robotModel.visible)
      .map((robotModel) => NUgusViewModel.of(robotModel).robot)
  }

  @computed
  private get fieldLineDots() {
    const group = new Object3D()
    this.model.robots.forEach((robot) =>
      robot.fieldLinesDots.rPWw.forEach((d) => {
        const mesh = new Mesh(
          LocalisationViewModel.fieldLineDotGeometry(),
          LocalisationViewModel.fieldLineDotMaterial(),
        )
        mesh.position.copy(d.add(new Vector3(0, 0, 0.005)).toThree())
        group.add(mesh)
      }),
    )
    return group
  }

  private static readonly fieldLineDotGeometry = circleBufferGeometry(() => ({ radius: 0.02, segments: 20 }))

  private static readonly fieldLineDotMaterial = meshBasicMaterial(() => ({ color: new Color('blue') }))
}
