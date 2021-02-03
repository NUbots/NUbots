import { observable } from 'mobx'

import { Appearance } from '../appearance/appearance'
import { BasicAppearance } from '../appearance/basic_appearance'

import { Geometry } from './geometry'

export type ShapeOpts<T extends Geometry> = {
  appearance: Appearance
  geometry: T
}

export class Shape<T extends Geometry> {
  @observable appearance: Appearance
  @observable geometry: T

  constructor(opts: ShapeOpts<T>) {
    this.appearance = opts.appearance
    this.geometry = opts.geometry
  }

  static of<T extends Geometry>(geometry: T, appearance: Appearance = BasicAppearance.of()) {
    return new Shape<T>({
      appearance,
      geometry,
    })
  }
}
