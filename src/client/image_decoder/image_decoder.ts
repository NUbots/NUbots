import { IAtom } from 'mobx'
import { createAtom } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { BufferGeometry } from 'three'
import { PlaneBufferGeometry } from 'three'
import { RawShaderMaterial } from 'three'
import { WebGLRenderTarget } from 'three'
import { Scene } from 'three'
import { Mesh } from 'three'
import { PixelFormat } from 'three'
import { Camera } from 'three'
import { OrthographicCamera } from 'three'
import { DataTexture } from 'three'
import { LuminanceFormat } from 'three'
import { RGBFormat } from 'three'
import { Texture } from 'three'
import { UnsignedByteType } from 'three'
import { ClampToEdgeWrapping } from 'three'
import { LinearFilter } from 'three'
import { NearestFilter } from 'three'
import { WebGLRenderer } from 'three'

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

export function fourccToString(code: number): string {
  return String.fromCharCode(code & 0xFF) +
  String.fromCharCode(code >> 8 & 0xFF) +
  String.fromCharCode(code >> 16 & 0xFF) +
  String.fromCharCode(code >> 24 & 0xFF)
}

export interface Image {
  readonly width: number
  readonly height: number
  readonly format: number
  readonly data: Uint8Array
}

export class ImageDecoder {

  private atom: IAtom

  // These two functions are used to cache/destroy textures as they are created
  private cache: {
    get(): Texture
    destroy(): void
  }

  // Variables needed by the bayer decoder
  private bayerDecoder?: { scene: Scene, camera: Camera, geometry: BufferGeometry, shader: RawShaderMaterial }

  constructor(private renderer: WebGLRenderer) {

    this.atom = createAtom('ImageDecoder', () => {}, this.destroy)

    this.cache = {
      get: () => new Texture(),
      destroy: () => {},
    }
  }

  static of = createTransformer((renderer: WebGLRenderer) => {
    return new ImageDecoder(renderer)
  }, decoder => decoder && decoder.destroy())

  private destroy = () => {
    this.cache.destroy()

    // Cleanup the bayer shader if we used it
    if (this.bayerDecoder) {
      this.bayerDecoder.geometry.dispose()
      this.bayerDecoder.shader.dispose()
    }
  }

  private updateCache(get: () => Texture, destroy: () => void) {
    // Destroy the previous texture
    this.cache.destroy()

    // Update the cache
    this.cache = { get, destroy }
  }

  // Update our stored texture
  update(image: Image): void {
    switch (image.format) {
      case fourcc('JPEG'): return this.jpeg(image)
      case fourcc('GRBG'): return this.bayer(image, [1, 0])
      case fourcc('RGGB'): return this.bayer(image, [0, 0])
      case fourcc('GBRG'): return this.bayer(image, [0, 1])
      case fourcc('BGGR'): return this.bayer(image, [1, 1])
      case fourcc('RGB3'): return this.dataTexture(image, RGBFormat)
      case fourcc('GREY'):
      case fourcc('GRAY'): return this.dataTexture(image, LuminanceFormat)
      default:
        throw Error(`Unsupported image format ${fourccToString(image.format)}`)
    }
  }

  get texture(): Texture {
    if (this.atom.reportObserved()) {
      return this.cache.get()
    } else {
      throw new Error('Atom accessed from a non reaction context')
    }
  }

  private bayer(image: Image, firstRed: [number, number]): void {

    // If we have never decoded bayer with this decoder before we need to make our threejs objects
    this.bayerDecoder = {
      scene: new Scene(),
      camera: new OrthographicCamera(-1, 1, 1, -1, 0, 1),
      geometry: new PlaneBufferGeometry(2, 2),
      shader: new RawShaderMaterial({
        vertexShader: String(bayerVertexShader),
        fragmentShader: String(bayerFragmentShader),
        depthTest: false,
        depthWrite: false,
      }),
    }

    const { width, height } = image
    const target = new WebGLRenderTarget(width, height)
    target.depthBuffer = false
    target.stencilBuffer = false

    // Create our getter function to debayer the first time it's called
    const get = () => {

      // The raw bayer texture
      const rawTexture = new DataTexture(
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
      rawTexture.needsUpdate = true

      // Cloning a material allows for new uniforms without recompiling the shader itself
      const material = this.bayerDecoder!.shader.clone()
      material.uniforms.sourceSize = { value: [width, height, 1 / width, 1 / height] }
      material.uniforms.firstRed = { value: firstRed }
      material.uniforms.image = { value: rawTexture }

      const mesh = new Mesh(this.bayerDecoder!.geometry, material)
      mesh.frustumCulled = false

      // Cleanup last render
      const scene = this.bayerDecoder!.scene
      scene.remove(...this.bayerDecoder!.scene.children)
      scene.add(mesh)

      this.renderer.render(scene, this.bayerDecoder!.camera, target)

      // We don't need the raw texture anymore since we already rendered the bayer
      rawTexture.dispose()

      // Now that we have debayered once, we can just keep returning the result
      this.cache.get = () => target.texture

      return target.texture
    }

    // Set our cache to use the getter function
    this.updateCache(get, () => target.dispose())

    // The image is ready to be read now
    this.atom.reportChanged()
  }

  private jpeg(image: Image): void {

    // Create a blob URL for our loaded data
    const blob = new Blob([image.data], { type: 'image/jpeg' })
    const url = window.URL.createObjectURL(blob)

    // Create our image tag and load the url in
    const tag = document.createElement('img')

    // Don't bother doing anything until the image loads
    tag.onload = () => {
      const texture = new Texture(tag)
      texture.needsUpdate = true
      texture.minFilter = LinearFilter
      texture.flipY = false

      this.updateCache(
        () => texture,
        () => {
          texture.dispose()
          window.URL.revokeObjectURL(url)
        },
      )

      this.atom.reportChanged()
    }
    tag.src = url
  }

  private dataTexture(image: Image, format: PixelFormat): void {
    this.updateCache(() => {
      const texture = new DataTexture(
          image.data,
          image.width,
          image.height,
          format,
          UnsignedByteType,
          Texture.DEFAULT_MAPPING,
          ClampToEdgeWrapping,
          ClampToEdgeWrapping,
          LinearFilter,
          LinearFilter,
        )
      texture.flipY = false
      texture.needsUpdate = true

        // Update our cache so we don't upload to the GPU each time, and we dispose of our texture
        // Don't use update cache though as we don't want to dispose of ourself
      this.cache.get = () => texture
      this.cache.destroy = () => {
        texture.dispose()
      }

      return texture
    },
      () => {},
    )

    // This is ready immediately
    this.atom.reportChanged()
  }
}
