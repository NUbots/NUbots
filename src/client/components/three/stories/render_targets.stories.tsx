import { storiesOf } from '@storybook/react'
import { IComputedValue } from 'mobx'
import { action } from 'mobx'
import { reaction } from 'mobx'
import { computed } from 'mobx'
import { observable } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { now } from 'mobx-utils'
import React from 'react'
import { WebGLRenderTarget } from 'three'
import { TextureLoader } from 'three'
import { LinearFilter } from 'three'
import { ClampToEdgeWrapping } from 'three'
import { Texture } from 'three'
import { Color } from 'three'

import { disposableComputed } from '../../../base/disposable_computed'
import { Vector3 } from '../../../math/vector3'
import { fullscreen } from '../../storybook/fullscreen'
import { planeGeometry } from '../builders'
import { pointLight } from '../builders'
import { ambientLight } from '../builders'
import { boxGeometry } from '../builders'
import { stage } from '../builders'
import { meshPhongMaterial } from '../builders'
import { perspectiveCamera } from '../builders'
import { renderTarget } from '../builders'
import { meshBasicMaterial } from '../builders'
import { mesh } from '../builders'
import { scene } from '../builders'
import { orthographicCamera } from '../builders'
import { Canvas } from '../three'
import { Three } from '../three'

import robotSvgUrl from './robot.file.svg'

storiesOf('component.three', module)
  .addDecorator(fullscreen)
  .add('renders static scene with render targets', () => {
    return <RenderTargetHarness/>
  })
  .add('renders animated scene with render targets', () => {
    return <RenderTargetHarness animate/>
  })

type Model = { time: number }

class RenderTargetHarness extends React.Component<{ animate?: boolean }> {
  @observable
  private readonly model: Model = {
    time: 0,
  }

  componentDidMount() {
    this.update(0)
    this.props.animate && disposeOnUnmount(this, reaction(() => now('frame'), this.update))
  }

  render() {
    return <Three stage={this.stage} clearColor={new Color('white')}/>
  }

  private stage = (canvas: Canvas) => {
    const robotRenderTarget = renderTarget(() => ({ width: 512, height: 512 }))
    const robotTexture = computed(() => robotRenderTarget.get().texture)
    const robotViewModel = RobotViewModel.of(this.model, robotRenderTarget)
    const innerBoxRenderTarget = renderTarget(() => ({ width: 512, height: 512 }))
    const innerBoxTexture = computed(() => innerBoxRenderTarget.get().texture)
    const innerBoxViewModel = OrangeBoxViewModel.of(this.model, robotTexture, innerBoxRenderTarget)
    const viewModel = WhiteBoxViewModel.of(canvas, this.model, innerBoxTexture)
    return computed(() => [robotViewModel.stage, innerBoxViewModel.stage, viewModel.stage])
  }

  @action.bound
  private update(now: number) {
    this.model.time = 2 * Math.PI * now / (60 * 1000)
  }
}

class WhiteBoxViewModel {
  constructor(
    private readonly canvas: Canvas,
    private readonly model: Model,
    private readonly orangeBoxTexture: IComputedValue<Texture>,
  ) {
  }

  static of(canvas: Canvas, model: Model, orangeBoxTexture: IComputedValue<Texture>) {
    return new WhiteBoxViewModel(canvas, model, orangeBoxTexture)
  }

  readonly stage = stage(() => ({
    camera: this.camera.get(),
    scene: this.scene.get(),
  }))

  private readonly camera = perspectiveCamera(() => ({
    fov: 60,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.5,
    far: 10,
    position: this.cameraPosition,
  }))

  @computed
  private get cameraPosition() {
    return Vector3.from({ x: 0, y: 0, z: 2 })
  }

  private readonly scene = scene(() => ({
    children: [
      this.ambientLight.get(),
      this.pointLight.get(),
      this.box.get(),
    ],
  }))

  private readonly ambientLight = ambientLight(() => ({ intensity: 0.5 }))

  private readonly pointLight = pointLight(() => ({
    intensity: 0.5,
    position: this.cameraPosition,
  }))

  private readonly box = mesh(() => ({
    geometry: this.geometry.get(),
    material: this.material.get(),
    rotation: new Vector3(3 * this.model.time, 5 * this.model.time, 7 * this.model.time),
  }))

  private readonly geometry = boxGeometry(() => ({ width: 1, height: 1, depth: 1 }))

  private readonly material = meshPhongMaterial(() => ({ map: this.orangeBoxTexture.get() }))
}

class OrangeBoxViewModel {
  constructor(
    private readonly model: Model,
    private readonly robotTexture: IComputedValue<Texture>,
    private readonly renderTarget: IComputedValue<WebGLRenderTarget>,
  ) {
  }

  static of(model: Model, robotTexture: IComputedValue<Texture>, renderTarget: IComputedValue<WebGLRenderTarget>) {
    return new OrangeBoxViewModel(model, robotTexture, renderTarget)
  }

  readonly stage = stage(() => ({
    scene: this.scene.get(),
    camera: this.camera.get(),
    target: this.renderTarget.get(),
  }))

  private readonly scene = scene(() => ({
    children: [
      this.ambientLight.get(),
      this.pointLight.get(),
      this.box.get(),
    ],
  }))

  private readonly ambientLight = ambientLight(() => ({ color: new Color('orange'), intensity: 0.5 }))

  private readonly pointLight = pointLight(() => ({
    color: new Color('orange'),
    intensity: 0.5,
    position: this.cameraPosition,
  }))

  private readonly camera = perspectiveCamera(() => ({
    fov: 60,
    aspect: 1,
    near: 0.5,
    far: 10,
    position: this.cameraPosition,
  }))

  @computed
  private get cameraPosition() {
    return Vector3.from({ x: 0, y: 0, z: 4 })
  }

  private readonly box = mesh(() => ({
    geometry: this.geometry.get(),
    material: this.material.get(),
    rotation: new Vector3(-17 * this.model.time, -13 * this.model.time, -11 * this.model.time),
  }))

  private readonly geometry = boxGeometry(() => ({ width: 2, height: 2, depth: 2 }))

  private readonly material = meshPhongMaterial(() => ({ map: this.robotTexture.get() }))
}

class RobotViewModel {
  constructor(
    private readonly model: Model,
    private readonly renderTarget: IComputedValue<WebGLRenderTarget>,
  ) {
  }

  static of(model: Model, renderTarget: IComputedValue<WebGLRenderTarget>) {
    return new RobotViewModel(model, renderTarget)
  }

  readonly stage = stage(() => ({
    scene: this.scene.get(),
    camera: this.camera.get(),
    target: this.renderTarget.get(),
  }))

  private readonly scene = scene(() => ({ children: [this.robot.get()] }))

  private readonly camera = orthographicCamera(() => ({ left: -1, right: 1, top: 1, bottom: -1, near: 0, far: 1 }))

  private readonly robot = mesh(() => ({
    geometry: this.geometry.get(),
    material: this.material.get(),
    rotation: new Vector3(0, 0, 17 * this.model.time),
  }))

  private readonly geometry = planeGeometry(() => ({ width: 1, height: 1 }))

  private readonly material = meshBasicMaterial(() => ({ map: this.texture.get(), transparent: true }))

  private readonly texture = disposableComputed<Texture>(() => {
    const texture = new TextureLoader().load(String(robotSvgUrl))
    texture.generateMipmaps = false
    texture.wrapS = texture.wrapT = ClampToEdgeWrapping
    texture.minFilter = LinearFilter
    texture.magFilter = LinearFilter
    return texture
  })
}
