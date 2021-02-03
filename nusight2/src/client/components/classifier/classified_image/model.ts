import { observable } from 'mobx'

import { Lut } from '../lut'

export type RawImage = { readonly type: 'image'; readonly image: HTMLImageElement }

export class ClassifiedImageModel {
  @observable.ref rawImage?: RawImage
  @observable.ref lut: Lut

  constructor({ lut }: { lut: Lut }) {
    this.lut = lut
  }

  static of({ lut }: { lut: Lut }) {
    return new ClassifiedImageModel({ lut })
  }
}
