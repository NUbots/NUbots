import { storiesOf } from '@storybook/react'
import { autorun } from 'mobx'
import { observable } from 'mobx'
import { observer } from 'mobx-react'
import { now } from 'mobx-utils'
import { useEffect } from 'react'
import { useState } from 'react'
import { useMemo } from 'react'
import React from 'react'
import * as THREE from 'three'

import { range } from '../../../../../shared/base/range'
import { Matrix4 } from '../../../../math/matrix4'
import { Vector2 } from '../../../../math/vector2'
import { Vector3 } from '../../../../math/vector3'
import { fullscreen } from '../../../storybook/fullscreen'
import { GreenHorizonView } from '../greenhorizon'
import { Lens } from '../model'
import { CameraParams } from '../model'
import { Projection } from '../model'
import { GreenHorizon } from '../model'
import { Canvas } from '@react-three/fiber'

storiesOf('components/vision2/camera/greenhorizon', module)
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

const GreenHorizonHarness = observer(({ animate }: { animate?: boolean }) => {
  const camera = useMemo(() => {
    const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -1, 1)
    ;(camera as any).manual = true
    return camera
  }, [])

  const [model] = useState(() => (
    observable({
      greenHorizon: new GreenHorizon({ horizon: generateHorizon(0), Hcw }),
      params: new CameraParams({
        Hcw,
        lens: new Lens({
          projection: Projection.RECTILINEAR,
          focalLength: 415 / 800,
        }),
      })
    })
  ))

  useEffect(() => (
    autorun(() => {
      const t = animate ? now('frame') / 1000 : 0
      model.greenHorizon.horizon = generateHorizon(t)
    })
  ))

  return <Canvas
    gl={{ antialias: false, alpha: false, depth: false, stencil: false }}
    orthographic={true}
    camera={camera}
    linear={true}
    flat={true}
    style={{ backgroundColor: 'black' }}
  >
    <GreenHorizonView greenHorizon={model.greenHorizon} params={model.params}/>
  </Canvas>
})

function generateHorizon(time: number): Vector3[] {
  const focalLength = 415 / 800
  const n = 50
  return range(n + 1).map(i => {
    const t = mod2pi(2 * Math.PI * (i / n) + time / 10)
    const p = lissajous(t).multiplyScalar(0.4) // Why lissajous? Why not.
    const rFCc = unprojectRectilinear(p, focalLength)
    const Rwc = new THREE.Matrix4().extractRotation(Hwc.toThree())
    return Vector3.fromThree(rFCc.toThree().applyMatrix4(Rwc)) // rFCw
  })
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
