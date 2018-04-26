import { observable } from 'mobx'

import { ImageModelOpts } from '../../../image_decoder/image_decoder'
import { ImageModel } from '../../../image_decoder/image_decoder'
import { Matrix4 } from '../../../math/matrix4'
import { VisionRobotModel } from '../model'

interface VisionImageModelOpts extends ImageModelOpts {
  Hcw: Matrix4
}

export class VisionImageModel extends ImageModel {

  @observable Hcw: Matrix4

  constructor(opts: VisionImageModelOpts) {
    super(opts)
    this.Hcw = opts.Hcw
  }

  static of(opts: VisionImageModelOpts) {
    return new ImageModel(opts)
  }
}

type CameraModelOpts = {
  id: number
  name: string
}

export class CameraModel {
  readonly id: number

  @observable image?: VisionImageModel
  @observable name: string

  constructor(private model: VisionRobotModel, { id, name }: CameraModelOpts) {
    this.id = id
    this.name = name
  }

  static of(model: VisionRobotModel, { id, name }: CameraModelOpts) {
    return new CameraModel(model, { id, name })
  }
}
