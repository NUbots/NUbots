import { observable } from 'mobx'

import { Matrix4 } from '../../../math/matrix4'
import { VisionRobotModel } from '../model'

type ImageModelOpts = {
  width: number
  height: number
  format: number
  data: Uint8Array
  Hcw: Matrix4
}

export class ImageModel {
  @observable width: number
  @observable height: number
  @observable format: number
  @observable.ref data: Uint8Array
  @observable Hcw: Matrix4

  constructor({ width, height, format, data, Hcw }: ImageModelOpts) {
    this.width = width
    this.height = height
    this.format = format
    this.data = data
    this.Hcw = Hcw
  }

  static of({ width, height, format, data, Hcw }: ImageModelOpts) {
    return new ImageModel({ width, height, format, data, Hcw })
  }
}

type CameraModelOpts = {
  id: number
  name: string
}

export class CameraModel {
  readonly id: number

  @observable image?: ImageModel
  @observable name: string

  constructor(private model: VisionRobotModel, { id, name }: CameraModelOpts) {
    this.id = id
    this.name = name
  }

  static of(model: VisionRobotModel, { id, name }: CameraModelOpts) {
    return new CameraModel(model, { id, name })
  }
}
