import { observable } from 'mobx'
import { Appearance } from '../appearance/appearance'
import { BasicAppearance } from '../appearance/basic_appearance'
import { Geometry } from '../geometry/geometry'

export class Shape<T> {
  @observable public geometry: T
  @observable public appearance: Appearance

  constructor(geometry: T, appearance: Appearance) {
    this.geometry = geometry
    this.appearance = appearance
  }

  public static of(geometry: Geometry, appearance: Appearance = BasicAppearance.of()) {
    return new Shape(geometry, appearance)
  }
}
