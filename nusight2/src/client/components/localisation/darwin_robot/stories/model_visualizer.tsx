import { computed } from 'mobx'
import { Component } from 'react'
import React from 'react'
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

export class ModelVisualiser extends Component<{
  model(): Object3D
  cameraPosition: Vector3
}> {
  render() {
    return <Three stage={this.stage} />
  }

  private stage = (canvas: Canvas) => {
    const viewModel = new ViewModel(canvas, this.props.model, this.props.cameraPosition)
    return computed(() => viewModel.stage)
  }
}

class ViewModel {
  private readonly canvas: Canvas
  private readonly model: () => Object3D
  private readonly cameraPosition: Vector3

  constructor(canvas: Canvas, model: () => Object3D, cameraPosition: Vector3) {
    this.canvas = canvas
    this.model = model
    this.cameraPosition = cameraPosition
  }

  @computed
  get stage(): Stage {
    return { camera: this.camera(), scene: this.scene() }
  }

  private readonly camera = perspectiveCamera(() => ({
    fov: 75,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.01,
    far: 100,
    position: this.cameraPosition,
    up: Vector3.from({ x: 0, y: 0, z: 1 }),
    lookAt: Vector3.of(),
  }))

  private readonly scene = scene(() => ({
    children: [this.helper, this.spotlight, this.pointlight, this.model()],
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
    light.position.copy(this.camera().position)
    return light
  }
}
