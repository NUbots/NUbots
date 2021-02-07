import { observable } from 'mobx'

import { Lut } from '../lut'

import { Dragger } from './controller'

export class VisualizerModel {
  @observable.ref lut: Lut
  @observable.shallow camera: { distance: number; elevation: number; azimuth: number }
  @observable.ref dragger?: Dragger

  constructor({
    lut,
    camera,
  }: {
    lut: Lut
    camera: { distance: number; elevation: number; azimuth: number }
  }) {
    this.lut = lut
    this.camera = camera
  }

  static of(lut: Lut): VisualizerModel {
    return new VisualizerModel({
      lut,
      camera: {
        distance: 4,
        elevation: 0,
        azimuth: 0,
      },
    })
  }
}
