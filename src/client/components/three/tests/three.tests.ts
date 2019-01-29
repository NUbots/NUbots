import { Camera } from 'three'
import { OrthographicCamera } from 'three'
import { Color } from 'three'
import { Object3D } from 'three'
import { PerspectiveCamera } from 'three'
import { Scene } from 'three'

import { StageCache } from '../three'

describe('StageCache', () => {
  let stage: StageCache
  let aCamera: Camera
  let aScene: Scene
  let aChild: Object3D

  beforeEach(() => {
    stage = new StageCache()
    aCamera = new PerspectiveCamera()
    aScene = new Scene()
    aChild = new Object3D()
  })

  it('copies scene properties', () => {
    aScene.background = new Color('red')
    aScene.add(aChild)
    const { scene } = stage.copy({ camera: aCamera, scene: aScene })
    expect(scene).toMatchObject({ background: new Color('red'), children: [aChild] })
    expect(scene).not.toBe(aChild)
    expect(scene.children[0]).toBe(aChild)
  })

  it('copies perspective camera properties', () => {
    const perspectiveCamera = new PerspectiveCamera(150, 2, 5, 10)
    const { camera } = stage.copy({ camera: perspectiveCamera, scene: aScene })
    expect(camera).not.toBe(perspectiveCamera)
    expect(camera).toBeInstanceOf(PerspectiveCamera)
    expect(camera).toMatchObject({ fov: 150, aspect: 2, near: 5, far: 10 })
  })

  it('copies orthographic camera properties', () => {
    const orthogonalCamera = new OrthographicCamera(-1, 1, 1, -1, 5, 10)
    const { camera } = stage.copy({ camera: orthogonalCamera, scene: aScene })
    expect(camera).not.toBe(orthogonalCamera)
    expect(camera).toBeInstanceOf(OrthographicCamera)
    expect(camera).toMatchObject({ left: -1, right: 1, top: 1, bottom: -1, near: 5, far: 10 })
  })

  it('returns the same instance after coping twice', () => {
    const firstCamera = new PerspectiveCamera(150, 2, 5, 10)
    const { camera: firstCopy } = stage.copy({ camera: firstCamera, scene: aScene })
    expect(firstCopy).toMatchObject({ fov: 150, aspect: 2, near: 5, far: 10 })

    const secondCamera = new PerspectiveCamera(50, 1, 2, 5)
    const { camera: secondCopy } = stage.copy({ camera: secondCamera, scene: aScene })
    expect(firstCopy).toBe(secondCopy)
    expect(secondCopy).toMatchObject({ fov: 50, aspect: 1, near: 2, far: 5 })
  })

  it('returns the same instance per camera type', () => {
    // Switch between perspective and orthogonal twice and make sure it caches per type.
    const firstCamera = new PerspectiveCamera(150, 2, 5, 10)
    const { camera: firstCopy } = stage.copy({ camera: firstCamera, scene: aScene })

    const secondCamera = new OrthographicCamera(-1, 1, 1, -1, 5, 10)
    const { camera: secondCopy } = stage.copy({ camera: secondCamera, scene: aScene })
    expect(firstCopy).not.toBe(secondCopy)

    const { camera: thirdCopy } = stage.copy({ camera: firstCamera, scene: aScene })
    expect(thirdCopy).toBe(firstCopy)

    const { camera: fourthCopy } = stage.copy({ camera: secondCamera, scene: aScene })
    expect(fourthCopy).toBe(secondCopy)
  })
})
