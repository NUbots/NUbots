import * as bounds from 'binary-search-bounds'
import { observable } from 'mobx'
import { computed } from 'mobx'
import { autorun } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { RawShaderMaterial } from 'three'
import { Float32BufferAttribute } from 'three'
import { Scene } from 'three'
import { WebGLRenderer } from 'three'
import { Mesh } from 'three'
import { BufferGeometry } from 'three'
import { Camera } from 'three'
import { OrthographicCamera } from 'three'
import { Vector2 } from 'three'

import { ImageDecoder } from '../../../image_decoder/image_decoder'

import { CameraModel } from './model'
import { VisualMesh } from './model'
import * as fragmentShader from './shaders/mesh.frag'
import * as vertexShader from './shaders/mesh.vert'

export class CameraViewModel {

  @observable.ref canvas: HTMLCanvasElement | null = null

  readonly camera: Camera
  readonly destroy: () => void

  constructor(
    private model: CameraModel,
    // We cache both the scene and the camera here as THREE.js uses these objects to store its own render lists.
    // So to conserve memory, it is best to keep them referentially identical across renders.
    private scene: Scene,
    camera: Camera,
  ) {
    this.camera = camera

    // Setup an autorun that will feed images to our image decoder when they change
    this.destroy = autorun(() => {
      this.canvas && this.decoder.update(this.model.image!)
    })
  }

  static of = createTransformer((model: CameraModel) => {
    return new CameraViewModel(
      model,
      new Scene(),
      new OrthographicCamera(-1, 1, 1, -1, 0, 1),
    )
  })

  @computed
  get id(): number {
    return this.model.id
  }

  @computed
  get name(): string {
    return this.model.name
  }

  @computed
  private get decoder() {
    return ImageDecoder.of(this.renderer(this.canvas)!)
  }

  renderer = createTransformer((canvas: HTMLCanvasElement | null) => {
    if (canvas) {
      return new WebGLRenderer({ canvas, alpha: true })
    }
  }, renderer => renderer && renderer.dispose())

  getScene(): Scene {
    const scene = this.scene
    scene.remove(...scene.children)
    if (this.model.mesh && this.model.image) {
      scene.add(this.visualMesh(this.model.mesh))
    }
    return scene
  }

  private visualMesh = createTransformer((mesh: VisualMesh): Mesh => {
    const material = this.meshMaterial
    material.uniforms.image.value = this.decoder.texture
    material.uniforms.dimensions.value = new Vector2(this.model.image!.width, this.model.image!.height)

    const obj = new Mesh(this.meshGeometry(mesh), this.meshMaterial)
    obj.frustumCulled = false
    return obj
  })

  @computed
  get meshMaterial(): RawShaderMaterial {
    return new RawShaderMaterial({
      vertexShader: String(vertexShader),
      fragmentShader: String(fragmentShader),
      uniforms: {
        image: { type: 't' },
        dimensions: { value: new Vector2() },
      },
    })
  }

  private meshGeometry = createTransformer((mesh: VisualMesh): BufferGeometry => {

    const { rows, indices, neighbours, coordinates, classifications } = mesh

    // Cumulative sum so we can work out which row our segments are on
    const cRows = rows.reduce((acc, v, i) => {
      acc.push(acc[i] + v)
      return acc
    }, [0])

    // Calculate our position
    const position = ([] as number[]).concat(...indices.map(i => {
      // Which ring we are on as a value between 0 and 1
      const idx = bounds.le(cRows, i)
      const phi = idx / rows.length
      // How far around the ring we are as a value between 0 and 1
      const theta = (i - cRows[idx]) / rows[idx]
      return [phi, theta]
    }), [Number.NaN, Number.NaN])

    // Calculate our triangle indexes
    const triangles = ([] as number[]).concat(...(neighbours.map((n, i) => {
      // Define our top left and top triangles (all that's needed since we get smaller with distance)
      return [
        i, n[0], n[2], // TL
        i, n[1], n[0], // T
      ]
    })))

    // Calculate our uv for mapping images
    const uvs = ([] as number[]).concat(...coordinates, [Number.NaN, Number.NaN])

    // Choose our classification to view
    const c = classifications[classifications.length - 1]

    const geometry = new BufferGeometry()
    geometry.setIndex(triangles)
    geometry.addAttribute('position', new Float32BufferAttribute(position, 2))
    geometry.addAttribute('classification', new Float32BufferAttribute(c.values, c.dim))
    geometry.addAttribute('uv', new Float32BufferAttribute(uvs, 2))

    return geometry
  }, (geometry?: BufferGeometry) => geometry && geometry.dispose())

}
