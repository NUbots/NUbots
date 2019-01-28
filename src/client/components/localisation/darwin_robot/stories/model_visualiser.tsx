import { IComputedValue } from 'mobx'
import { computed } from 'mobx'
import { expr } from 'mobx-utils'
import { Component } from 'react'
import * as React from 'react'
import { PerspectiveCamera } from 'three'
import { SpotLight } from 'three'
import { PointLight } from 'three'
import { Object3D } from 'three'
import { AxesHelper } from 'three'
import { Scene } from 'three'

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
    return { camera: this.camera, scene: this.scene }
  }

  @computed
  private get camera() {
    const aspect = expr(() => this.canvas.width / this.canvas.height)
    const camera = new PerspectiveCamera(75, aspect, 0.01, 100)
    camera.position.set(0.4, 0.3, 0.3)
    camera.up.set(0, 0, 1)
    camera.lookAt(0, 0, 0)
    return camera
  }

  @computed
  private get scene(): Scene {
    const scene = new Scene()
    scene.add(this.helper)
    scene.add(this.spotlight)
    scene.add(this.pointlight)
    scene.add(this.model.get())
    return scene
  }

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
    light.position.copy(this.camera.position)
    return light
  }
}
