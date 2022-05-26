import { storiesOf } from '@storybook/react'
import { computed } from 'mobx'
import { reaction } from 'mobx'
import { disposeOnUnmount } from 'mobx-react'
import { now } from 'mobx-utils'
import { Component } from 'react'
import React from 'react'
import * as THREE from 'three'

import { range } from '../../../../../shared/base/range'
import { Matrix4 } from '../../../../math/matrix4'
import { Vector2 } from '../../../../math/vector2'
import { Vector3 } from '../../../../math/vector3'
import { fullscreen } from '../../../storybook/fullscreen'
import { scene } from '../../../three/builders'
import { orthographicCamera } from '../../../three/builders'
import { stage } from '../../../three/builders'
import { Canvas } from '../../../three/three'
import { Three } from '../../../three/three'
import { GreenHorizonViewModel } from '../greenhorizon'
import { Lens } from '../model'
import { CameraParams } from '../model'
import { Projection } from '../model'
import { GreenHorizon } from '../model'

storiesOf('components.vision.camera.greenhorizon', module)
  .addDecorator(fullscreen)
  .add('Renders statically', () => <GreenHorizonHarness />)
  .add('Renders animated', () => <GreenHorizonHarness animate />)

const camHeight = 0.8
const Hwc = Matrix4.fromThree(
  new THREE.Matrix4()
    .makeTranslation(0, 0, camHeight)
    .multiply(new THREE.Matrix4().makeRotationY(Math.PI / 4)),
)
const Hcw = Matrix4.fromThree(new THREE.Matrix4().getInverse(Hwc.toThree()))

class GreenHorizonHarness extends Component<{ animate?: boolean }> {
  render() {
    return <Three stage={this.stage} objectFit={{ type: 'contain', aspect: 1 }} />
  }

  private readonly stage = (canvas: Canvas) => {
    const viewModel = this.createViewModel(canvas)
    return computed(() => [viewModel.stage])
  }

  private createViewModel(canvas: Canvas): ViewModel {
    const greenHorizon = new GreenHorizon({ horizon: this.generateHorizon(0), Hcw })
    const params = new CameraParams({
      Hcw,
      lens: new Lens({
        projection: Projection.RECTILINEAR,
        focalLength: 415 / 800,
      }),
    })
    this.props.animate &&
      disposeOnUnmount(
        this,
        reaction(
          () => now('frame') / 1000,
          t => (greenHorizon.horizon = this.generateHorizon(t)),
        ),
      )
    return ViewModel.of(canvas, greenHorizon, params)
  }

  private generateHorizon(time: number): Vector3[] {
    const focalLength = 415 / 800
    const n = 1000
    return range(n + 1).map(i => {
      const t = mod2pi(2 * Math.PI * (i / n) + time / 10)
      const p = lissajous(t).multiplyScalar(0.4) // Why lissajous? Why not.
      const rFCc = unprojectRectilinear(p, focalLength)
      const Rwc = new THREE.Matrix4().extractRotation(Hwc.toThree())
      return Vector3.fromThree(rFCc.toThree().applyMatrix4(Rwc)) // rFCw
    })
  }
}

class ViewModel {
  private readonly viewModel: GreenHorizonViewModel

  constructor(viewModel: GreenHorizonViewModel) {
    this.viewModel = viewModel
  }

  static of(canvas: Canvas, greenHorizon: GreenHorizon, params: CameraParams) {
    return new ViewModel(GreenHorizonViewModel.of(canvas, greenHorizon, params))
  }

  readonly stage = stage(() => ({ camera: this.camera(), scene: this.scene() }))

  private readonly camera = orthographicCamera(() => ({
    left: -1,
    right: 1,
    top: 1,
    bottom: -1,
    near: 0,
    far: 1,
  }))

  private readonly scene = scene(() => ({ children: [this.viewModel.greenhorizon()] }))
}

function unprojectRectilinear(point: Vector2, focalLength: number): Vector3 {
  return new Vector3(focalLength, point.x, point.y).normalize()
}

function lissajous(t: number, a = 3, b = 4) {
  // https://en.wikipedia.org/wiki/Lissajous_curve
  return new Vector2(Math.sin(a * t), Math.sin(b * t))
}

const mod =
  (n: number) =>
  (x: number): number =>
    ((x % n) + n) % n
const mod2pi = mod(2 * Math.PI)
