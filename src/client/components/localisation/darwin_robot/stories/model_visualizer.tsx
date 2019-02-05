import { IComputedValue } from 'mobx'
import { computed } from 'mobx'
import { Component } from 'react'
import * as React from 'react'
import { SpotLight } from 'three'
import { PointLight } from 'three'
import { Object3D } from 'three'
import { AxesHelper } from 'three'

import { Vector3 } from '../../../../math/vector3'
import { scene } from '../../../three/builders'
import { perspectiveCamera } from '../../../three/builders'
import { Stage } from '../../../three/three'
import { Canvas } from '../../../three/three'
import { Three } from '../../../three/three'

export class ModelVisualiser extends Component<{ model: IComputedValue<Object3D>; }> {
  render() {
    return <Three stage={this.stage}/>
  }

  private stage = (canvas: Canvas) => {
    const viewModel = new ViewModel(canvas, this.props.model)
    return computed(() => viewModel.stage)
  }
}

class ViewModel {
  constructor(private readonly canvas: Canvas, private readonly model: IComputedValue<Object3D>) {
  }

  @computed
  get stage(): Stage {
    return { camera: this.camera.get(), scene: this.scene.get() }
  }

  private readonly camera = perspectiveCamera(() => ({
    fov: 75,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.01,
    far: 100,
    position: Vector3.from({ x: 0.4, y: 0.3, z: 0.3 }),
    up: Vector3.from({ x: 0, y: 0, z: 1 }),
    lookAt: Vector3.of(),
  }))

  private readonly scene = scene(() => ({
    children: [
      this.helper,
      this.spotlight,
      this.pointlight,
      this.model.get(),
    ],
  }))

  @computed
  private get helper() {
    return new AxesHelper()
  }

  @computed
  private get spotlight() {
    const light = new SpotLight('#fff', 1, 20, Math.PI / 8)
    light.position.set(0, 0, 1)
    return light
  }

  @computed
  private get pointlight() {
    const light = new PointLight('#fff')
    light.position.copy(this.camera.get().position)
    return light
  }
}
