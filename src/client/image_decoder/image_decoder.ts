import { createTransformer } from 'mobx-utils'
import { BufferGeometry } from 'three'
import { PlaneBufferGeometry } from 'three'
import { RawShaderMaterial } from 'three'
import { WebGLRenderTarget } from 'three'
import { Scene } from 'three'
import { Mesh } from 'three'
import { WebGLRenderer } from 'three'
import { OrthographicCamera } from 'three'
import { DataTexture } from 'three'
import { LuminanceFormat } from 'three'
import { RGBFormat } from 'three'
import { Texture } from 'three'
import { UnsignedByteType } from 'three'
import { ClampToEdgeWrapping } from 'three'
import { LinearFilter } from 'three'
import { NearestFilter } from 'three'
import { Camera } from 'three'

import * as bayerFragmentShader from './shaders/bayer.frag'
import * as bayerVertexShader from './shaders/bayer.vert'

/**
 * Convert a four letter string into its integer fourcc code (see http://fourcc.org/)
 * This code allows identification of a stream using the integer.
 *
 * @param code four letters that describe the format
 *
 * @return the fourcc integer code for this format
 */
export function fourcc(code: string): number {
  return code.charCodeAt(3) << 24 | code.charCodeAt(2) << 16 | code.charCodeAt(1) << 8 | code.charCodeAt(0)
}

export interface Image {
  readonly width: number
  readonly height: number
  readonly format: number
  readonly data: Uint8Array
}

export class ImageDecoder {

  constructor(private renderer: WebGLRenderer,
              private scene: Scene,
              private camera: Camera,
              private geometry: BufferGeometry) {
  }

  static of = createTransformer((renderer: WebGLRenderer) => {
    return new ImageDecoder(
      renderer,
      new Scene(),
      new OrthographicCamera(-1, 1, 1, -1, 0, 1),
      new PlaneBufferGeometry(2, 2),
    )
  })

  decode = createTransformer((image: Image) => {
    switch (image.format) {
      case fourcc('GRBG'):
      case fourcc('RGGB'):
      case fourcc('GBRG'):
      case fourcc('BGGR'):
        return this.bayerTexture(image).texture
      case fourcc('RGB3'):
        return this.rgbTexture(image)
      case fourcc('GREY'):
      case fourcc('GRAY'):
        return this.grayTexture(image)
      default:
        throw Error(`Unsupported image format ${image.format}`)
    }
  })

  // By making this a transformer, we ensure that when we are no longer observed, the material is cleaned up
  private bayerShader = createTransformer((renderer: WebGLRenderer): RawShaderMaterial => {
    return new RawShaderMaterial({
      vertexShader: String(bayerVertexShader),
      fragmentShader: String(bayerFragmentShader),
      depthTest: false,
      depthWrite: false,
    })
  }, (material?: RawShaderMaterial) => material && material.dispose())

  private bayerTexture = createTransformer((image: Image) => {
    let firstRed
    switch (image.format) {
      case fourcc('GRBG'):
        firstRed = [1, 0]
        break
      case fourcc('RGGB'):
        firstRed = [0, 0]
        break
      case fourcc('GBRG'):
        firstRed = [0, 1]
        break
      case fourcc('BGGR'):
        firstRed = [1, 1]
        break
    }

    const { width, height } = image
    const renderTarget = new WebGLRenderTarget(width, height)
    renderTarget.depthBuffer = false
    renderTarget.stencilBuffer = false

    // Cloning a material allows for new uniforms without recompiling the shader itself
    const material = this.bayerShader(this.renderer).clone()
    material.uniforms.sourceSize = { value: [width, height, 1 / width, 1 / height] }
    material.uniforms.firstRed = { value: firstRed }
    material.uniforms.image = { value: this.rawTexture(image) }

    const mesh = new Mesh(this.geometry, material)
    mesh.frustumCulled = false

    // Cleanup last render
    const scene = this.scene
    scene.remove(...this.scene.children)
    scene.add(mesh)

    this.renderer.render(scene, this.camera, renderTarget)
    return renderTarget
  }, (target?: WebGLRenderTarget) => target && target.dispose())

  private rgbTexture = createTransformer((image: Image) => {
    const texture = new DataTexture(
      image.data,
      image.width,
      image.height,
      RGBFormat,
      UnsignedByteType,
      Texture.DEFAULT_MAPPING,
      ClampToEdgeWrapping,
      ClampToEdgeWrapping,
      LinearFilter,
      LinearFilter,
    )
    texture.flipY = false
    texture.needsUpdate = true
    return texture
  }, texture => texture && texture.dispose())

  private grayTexture = createTransformer((image: Image) => {
    const texture = new DataTexture(
      image.data,
      image.width,
      image.height,
      LuminanceFormat,
      UnsignedByteType,
      Texture.DEFAULT_MAPPING,
      ClampToEdgeWrapping,
      ClampToEdgeWrapping,
      LinearFilter,
      LinearFilter,
    )
    texture.flipY = false
    texture.needsUpdate = true
    return texture
  }, texture => texture && texture.dispose())

  private rawTexture = createTransformer((image: Image) => {
    const texture = new DataTexture(
      image.data,
      image.width,
      image.height,
      LuminanceFormat,
      UnsignedByteType,
      Texture.DEFAULT_MAPPING,
      ClampToEdgeWrapping,
      ClampToEdgeWrapping,
      NearestFilter,
      NearestFilter,
    )
    texture.needsUpdate = true
    return texture
  }, texture => texture && texture.dispose())
}
