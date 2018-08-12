import { observable } from 'mobx'

export class Quaternion {
  @observable x: number
  @observable y: number
  @observable z: number
  @observable w: number

  constructor(x: number, y: number, z: number, w: number) {
    this.x = x
    this.y = y
    this.z = z
    this.w = w
  }

  static of() {
    return new Quaternion(0, 0, 0, 1)
  }

  set(x: number, y: number, z: number, w: number): Quaternion {
    this.x = x
    this.y = y
    this.z = z
    this.w = w
    return this
  }
}
