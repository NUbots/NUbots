import { computed } from 'mobx'
import { LuminanceFormat } from 'three'
import { LinearFilter } from 'three'
import { ClampToEdgeWrapping } from 'three'
import { Texture } from 'three'
import { UnsignedByteType } from 'three'
import { RGBAFormat } from 'three'
import { Matrix4 } from 'three'
import { PlaneGeometry } from 'three'

import { disposableComputed } from '../../../base/disposable_computed'
import { dataTexture } from '../../three/builders'
import { shaderMaterial } from '../../three/builders'
import { shader } from '../../three/builders'
import { imageTexture } from '../../three/builders'
import { mesh } from '../../three/builders'
import { scene } from '../../three/builders'
import { orthographicCamera } from '../../three/builders'
import { Stage } from '../../three/three'

import { ClassifiedImageModel } from './model'
import fragmentShader from './shaders/classify.frag'
import vertexShader from './shaders/classify.vert'

export class ClassifiedImageViewModel {
  private readonly model: ClassifiedImageModel

  constructor(model: ClassifiedImageModel) {
    this.model = model
  }

  static of(model: ClassifiedImageModel) {
    return new ClassifiedImageViewModel(model)
  }

  get stage(): Stage {
    return { camera: this.camera(), scene: this.scene() }
  }

  private readonly camera = orthographicCamera(() => ({
    left: 0,
    right: 1,
    top: 1,
    bottom: 0,
    near: 0,
    far: 1,
  }))

  private readonly scene = scene(() => ({
    children: [this.image()],
  }))

  private readonly image = mesh(() => ({
    geometry: ClassifiedImageViewModel.geometry.get(),
    material: this.material(),
  }))

  private static geometry = disposableComputed(() => {
    const geometry = new PlaneGeometry(1, 1)
    geometry.applyMatrix(new Matrix4().makeTranslation(0.5, 0.5, 0))
    return geometry
  })

  private readonly material = shaderMaterial(() => ({
    shader: this.shader,
    uniforms: {
      image: { value: this.imageTexture() },
      lut: { value: this.lutTexture() },
      lutSize: { value: this.lutSize },
      bitsX: { value: this.model.lut.size.x },
      bitsY: { value: this.model.lut.size.y },
      bitsZ: { value: this.model.lut.size.z },
    },
  }))

  private readonly shader = shader(() => ({ vertexShader, fragmentShader }))

  private readonly imageTexture = imageTexture(() => ({
    image: this.imageElement,
    format: RGBAFormat,
    type: UnsignedByteType,
    mapping: Texture.DEFAULT_MAPPING,
    wrapS: ClampToEdgeWrapping,
    wrapT: ClampToEdgeWrapping,
    magFilter: LinearFilter,
    minFilter: LinearFilter,
    flipY: true,
  }))

  @computed
  private get imageElement(): HTMLImageElement | undefined {
    const rawImage = this.model.rawImage
    return rawImage && rawImage.type === 'image' ? rawImage.image : undefined
  }

  private readonly lutTexture = dataTexture(() => ({
    data: this.model.lut.data,
    width: this.lutSize,
    height: this.lutSize,
    format: LuminanceFormat,
    type: UnsignedByteType,
    mapping: Texture.DEFAULT_MAPPING,
    wrapS: ClampToEdgeWrapping,
    wrapT: ClampToEdgeWrapping,
    magFilter: LinearFilter,
    minFilter: LinearFilter,
    flipY: false,
  }))

  @computed
  get lutSize(): number {
    return Math.ceil(Math.sqrt(this.model.lut.data.length))
  }
}
