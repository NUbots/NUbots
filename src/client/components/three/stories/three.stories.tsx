import { storiesOf } from '@storybook/react'
import { IComputedValue } from 'mobx'
import { reaction } from 'mobx'
import { observable } from 'mobx'
import { action } from 'mobx'
import { computed } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { now } from 'mobx-utils'
import { Component } from 'react'
import * as React from 'react'
import { Light } from 'three'
import { Object3D } from 'three'
import { Material } from 'three'
import { Geometry } from 'three'
import { Camera } from 'three'
import { PointLight } from 'three'
import { MeshPhongMaterial } from 'three'
import { Mesh } from 'three'
import { BoxGeometry } from 'three'
import { PerspectiveCamera } from 'three'
import { Scene } from 'three'

import { Vector2 } from '../../../math/vector2'
import { Vector3 } from '../../../math/vector3'
import { Stage } from '../three'
import { Canvas } from '../three'
import { Three } from '../three'

type BoxModel = { color: string, size: number, position: Vector3, rotation: Vector3 }

class BoxVisualiser extends Component<{ animate?: boolean }> {
  @observable
  private model = {
    boxes: [
      { color: 'red', size: 1, position: Vector3.of(), rotation: Vector3.of() },
      { color: 'green', size: 1, position: Vector3.of(), rotation: Vector3.of() },
      { color: 'blue', size: 1, position: Vector3.of(), rotation: Vector3.of() },
    ],
  }

  componentDidMount() {
    this.update(0)
    this.props.animate && disposeOnUnmount(this, reaction(() => now('frame'), this.update))
  }

  @action.bound
  update(now: number) {
    const t = 2 * Math.PI * now / (20 * 1000)
    const n = this.model.boxes.length
    this.model.boxes.forEach((box, i) => {
      const position = Vector2.fromPolar(1, i * 2 * Math.PI / n + t)
      box.position.set(position.x, position.y, 0)
      box.rotation.set(Math.cos(3 * t + i), Math.cos(5 * t + i), Math.cos(7 * t + i))
    })
  }

  render() {
    return <Three createStage={this.createStage}/>
  }

  private createStage = (canvas: Canvas): IComputedValue<Stage> => {
    const geometry = computed(() => this.boxGeometry())
    const box = (box: BoxModel) => {
      const material = computed(() => this.boxMaterial(box))
      return computed(() => this.box(box, geometry.get(), material.get()))
    }
    const camera = computed(() => this.camera(canvas))
    const light = computed(() => this.light(camera.get()))
    const boxes = computed(() => this.model.boxes.map(b => box(b).get()))
    const scene = computed(() => this.scene([...boxes.get(), light.get()]))
    return computed(() => ({ camera: camera.get(), scene: scene.get() }))
  }

  private light(camera: Camera): Light {
    const light = new PointLight()
    light.position.copy(camera.position)
    return light
  }

  private camera(canvas: Canvas): Camera {
    const camera = new PerspectiveCamera(60, canvas.width / canvas.height, 0.5, 10)
    camera.position.z = 4
    return camera
  }

  private scene(children: Object3D[]): Scene {
    const scene = new Scene()
    scene.add(...children)
    return scene
  }

  private box(box: BoxModel, geometry: Geometry, material: Material): Mesh {
    const mesh = new Mesh(geometry, material)
    mesh.position.set(box.position.x, box.position.y, box.position.z)
    mesh.rotation.set(box.rotation.x, box.rotation.y, box.rotation.z)
    mesh.scale.setScalar(box.size)
    return mesh
  }

  private boxGeometry(): Geometry {
    return new BoxGeometry(1, 1, 1)
  }

  private boxMaterial(box: BoxModel): Material {
    return new MeshPhongMaterial({ color: box.color })
  }
}

const fullscreen = { width: 'calc(100vw - 20px)', height: 'calc(100vh - 20px)' }
storiesOf('component.three', module)
  .add('renders static scene', () => {
    return <div style={fullscreen}>
      <BoxVisualiser/>
    </div>
  })
  .add('renders animated scene', () => {
    return <div style={fullscreen}>
      <BoxVisualiser animate/>
    </div>
  })
