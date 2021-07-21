import { computed } from 'mobx'
import { createTransformer } from 'mobx-utils'
import { Geometry } from 'three'
import * as THREE from 'three'

import { UnreachableError } from '../../../../shared/base/unreachable_error'
import { Vector3 } from '../../../math/vector3'
import { imageTexture } from '../../three/builders'
import { dataTexture } from '../../three/builders'
import { meshBasicMaterial } from '../../three/builders'
import { shaderMaterial } from '../../three/builders'
import { rawShader } from '../../three/builders'
import { mesh } from '../../three/builders'
import { planeGeometry } from '../../three/builders'
import { ImageFormat } from '../image'
import { BayerImageFormat } from '../image'
import { ElementImage } from '../image'
import { DataImage } from '../image'
import { Image } from '../image'

import fragmentShader from './shaders/bayer.frag'
import vertexShader from './shaders/bayer.vert'

export class ImageViewModel {
  private readonly source: Image
  private readonly geometry: () => Geometry

  constructor(source: Image, geometry: () => Geometry) {
    this.source = source
    this.geometry = geometry
  }

  static of(source: Image) {
    return new ImageViewModel(source, ImageViewModel.geometry)
  }

  private static geometry = planeGeometry(() => ({ width: 2, height: 2 }))

  readonly image = mesh(() => ({
    geometry: this.geometry(),
    material: this.material,
    // Normally this effect could be achieved by setting texture.flipY to make
    // the textures the correct way up again. However this is ignored on RenderTargets
    // We can't flip it at the raw stage either as this would invert things like the Bayer pattern.
    // Instead we just leave everything flipped and correct it here by scaling by -1 on the y axis
    scale: new Vector3(1, -1, 1),
  }))

  @computed
  private get material(): THREE.Material {
    const { format } = this.source
    switch (format) {
      case ImageFormat.JPEG:
      case ImageFormat.RGB8:
      case ImageFormat.GREY:
      case ImageFormat.GRAY:
      case ImageFormat.Y8__:
        return this.basicMaterial()
      case ImageFormat.JPBG:
      case ImageFormat.JPRG:
      case ImageFormat.JPGR:
      case ImageFormat.JPGB:
      case ImageFormat.GRBG:
      case ImageFormat.RGGB:
      case ImageFormat.GBRG:
      case ImageFormat.BGGR:
      case ImageFormat.PJBG:
      case ImageFormat.PJRG:
      case ImageFormat.PJGR:
      case ImageFormat.PJGB:
        return this.bayerMaterial(this.firstRed(format), this.mosaicSize(format))
      default:
        throw new UnreachableError(format)
    }
  }

  private readonly basicMaterial = meshBasicMaterial(() => ({ map: this.texture }))

  private readonly bayerMaterial = shaderMaterial(
    (firstRed: [number, number], mosaicSize: number) => ({
      shader: this.bayerShader,
      depthTest: false,
      uniforms: {
        image: { value: this.texture },
        sourceSize: {
          value: [
            this.source.width,
            this.source.height,
            1 / this.source.width,
            1 / this.source.height,
          ],
        },
        firstRed: { value: firstRed },
        mosaicSize: { value: mosaicSize },
      },
    }),
  )

  private readonly bayerShader = rawShader(() => ({ vertexShader, fragmentShader }))

  @computed
  private get texture() {
    switch (this.source.type) {
      case 'data':
        return this.dataTexture(this.source)
      case 'element':
        return this.elementTexture(this.source)
      default:
        throw new UnreachableError(this.source)
    }
  }

  private readonly elementTexture = imageTexture(
    ({ element, width, height, format }: ElementImage) => ({
      image: element,
      width,
      height,
      format: this.textureFormat(format),
      magFilter: THREE.LinearFilter,
      minFilter: THREE.LinearFilter,
    }),
  )

  private readonly dataTexture = dataTexture(({ data, width, height, format }: DataImage) => ({
    data: data.get(),
    width,
    height,
    format: this.textureFormat(format),
    magFilter: THREE.LinearFilter,
    minFilter: THREE.LinearFilter,
  }))

  private textureFormat = createTransformer((format: ImageFormat): THREE.PixelFormat => {
    switch (format) {
      case ImageFormat.JPEG:
      case ImageFormat.JPBG:
      case ImageFormat.JPRG:
      case ImageFormat.JPGR:
      case ImageFormat.JPGB:
      case ImageFormat.PJBG:
      case ImageFormat.PJRG:
      case ImageFormat.PJGR:
      case ImageFormat.PJGB:
        return THREE.RGBAFormat
      case ImageFormat.RGB8:
        return THREE.RGBFormat
      case ImageFormat.GRBG:
      case ImageFormat.RGGB:
      case ImageFormat.GBRG:
      case ImageFormat.BGGR:
      case ImageFormat.GREY:
      case ImageFormat.GRAY:
      case ImageFormat.Y8__:
        return THREE.LuminanceFormat
      default:
        throw new UnreachableError(format)
    }
  })

  private firstRed = createTransformer((format: BayerImageFormat): [number, number] => {
    switch (format) {
      case ImageFormat.JPBG:
      case ImageFormat.BGGR:
      case ImageFormat.PJBG:
        return [1, 1]
      case ImageFormat.JPGR:
      case ImageFormat.GRBG:
      case ImageFormat.PJGR:
        return [1, 0]
      case ImageFormat.JPGB:
      case ImageFormat.GBRG:
      case ImageFormat.PJGB:
        return [0, 1]
      case ImageFormat.JPRG:
      case ImageFormat.RGGB:
      case ImageFormat.PJRG:
        return [0, 0]
      default:
        throw new UnreachableError(format)
    }
  })

  private mosaicSize = createTransformer((format: BayerImageFormat): number => {
    switch (format) {
      case ImageFormat.BGGR:
      case ImageFormat.RGGB:
      case ImageFormat.GRBG:
      case ImageFormat.GBRG:
        return 1 // One value per width/height
      case ImageFormat.JPBG:
      case ImageFormat.JPRG:
      case ImageFormat.JPGR:
      case ImageFormat.JPGB:
        return 2 // Two values per width/height
      case ImageFormat.PJBG:
      case ImageFormat.PJRG:
      case ImageFormat.PJGR:
      case ImageFormat.PJGB:
        return 4 // Four values per width/height
      default:
        throw new UnreachableError(format)
    }
  })
}
