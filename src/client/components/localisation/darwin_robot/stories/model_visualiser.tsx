import { IComputedValue } from 'mobx'
import { computed } from 'mobx'
import { Component } from 'react'
import * as React from 'react'
import { Scene } from 'three'
import { SpotLight } from 'three'
import { PointLight } from 'three'
import { Object3D } from 'three'
import { AxesHelper } from 'three'
import { PerspectiveCamera } from 'three'
import { Camera } from 'three'

import { Canvas } from '../../../three/three'
import { Three } from '../../../three/three'

export class ModelVisualiser extends Component<{ model: IComputedValue<Object3D>; }> {
  render() {
    return <Three createStage={this.createStage}/>
  }

  private createStage = (canvas: Canvas) => {
    const model = computed(() => this.props.model.get())
    const camera = computed(() => this.camera(canvas))
    const helper = computed(() => this.helper())
    const spotlight = computed(() => this.spotlight())
    const pointlight = computed(() => this.pointlight(camera.get()))
    const scene = computed(() => this.scene([helper.get(), spotlight.get(), pointlight.get(), model.get()]))
    return computed(() => ({ camera: camera.get(), scene: scene.get() }))
  }

  private camera(canvas: Canvas) {
    const camera = new PerspectiveCamera(75, canvas.width / canvas.height, 0.01, 100)
    camera.position.set(0.4, 0.3, 0.3)
    camera.up.set(0, 0, 1)
    camera.lookAt(0, 0, 0)
    return camera
  }

  private scene(children: Object3D[]): Scene {
    const scene = new Scene()
    scene.add(...children)
    return scene
  }

  private helper() {
    return new AxesHelper()
  }

  private spotlight() {
    const light = new SpotLight('#fff', 1, 20, Math.PI / 8)
    light.position.set(0, 0, 1)
    return light
  }

  private pointlight(camera: Camera) {
    const light = new PointLight('#fff')
    light.position.copy(camera.position)
    return light
  }
}
