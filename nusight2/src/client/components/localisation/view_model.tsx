import { computed } from 'mobx'
import '@react-three/fiber'
import React from 'react'
import { Color } from 'three'
import { Mesh } from 'three'
import { HemisphereLight } from 'three'
import { PointLight } from 'three'
import { Object3D } from 'three'

import { Vector3 } from '../../../shared/math/vector3'
import { meshBasicMaterial } from '../three/builders'
import { circleBufferGeometry } from '../three/builders'
import { scene } from '../three/builders'
import { perspectiveCamera } from '../three/builders'
import { Canvas } from '../three/three'
import { PerspectiveCamera } from '../three/three_fiber'

import { FieldView } from './field/view'
import { LocalisationModel } from './model'
import { NUgusViewModel } from './nugus_robot/view_model'
import { SkyboxViewModel } from './skybox/view_model'

export class LocalisationViewModel extends React.Component<{
  model: LocalisationModel
}> {
  render() {
    return <object3D>
      <PerspectiveCamera
        args={[75, 1, 0.01, 100]}
        position={this.model.camera.position.toArray()}
        rotation={[Math.PI / 2 + this.model.camera.pitch, 0, -Math.PI / 2 + this.model.camera.yaw, 'ZXY']}
        up={Vector3.from({ x: 0, y: 0, z: 1 }).toArray()}
      />
      <FieldView model={this.model.field}/>
      <SkyboxViewModel model={this.model.skybox}/>
    </object3D>
  }

  private get canvas(): Canvas {
    return {
      width: 320,
      height: 320,
    }
  }

  private get model() {
    return this.props.model
  }

  private readonly scene = scene(() => ({
    children: [...this.robots, this.hemisphereLight, this.pointLight, this.fieldLineDots],
  }))

  @computed
  private get robots(): Object3D[] {
    return this.model.robots
      .filter((robotModel) => robotModel.visible)
      .map((robotModel) => NUgusViewModel.of(robotModel).robot)
  }

  @computed
  private get hemisphereLight(): HemisphereLight {
    return new HemisphereLight('#fff', '#fff', 0.6)
  }

  @computed
  private get pointLight() {
    const light = new PointLight('#fff')
    light.position.set(this.model.camera.position.x, this.model.camera.position.y, this.model.camera.position.z)
    return light
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
