import { IAtom } from 'mobx'
import { createAtom } from 'mobx'
import { observable } from 'mobx'

import { Classification } from './classifications'

export class Lut {
  private atom: IAtom
  private rawData: Uint8Array
  @observable.shallow size: { x: number; y: number; z: number }

  constructor({
    atom,
    data,
    size,
  }: {
    atom: IAtom
    data: Uint8Array
    size: { x: number; y: number; z: number }
  }) {
    this.atom = atom
    this.rawData = data
    this.size = size
  }

  static of(size: { x: number; y: number; z: number }) {
    return new Lut({
      atom: createAtom('Lut'),
      data: new Uint8Array(2 ** (size.x + size.y + size.z)).fill(Classification.Unclassified),
      size,
    })
  }

  static generate(
    { x, y, z }: { x: number; y: number; z: number },
    fill?: (index: number) => number,
  ) {
    const n = 2 ** (x + y + z)
    const data = new Uint8Array(n)
    if (fill) {
      for (let i = 0; i < n; i++) {
        data[i] = fill(i)
      }
    } else {
      data.fill(Classification.Unclassified)
    }
    return new Lut({ atom: createAtom('Lut'), data, size: { x, y, z } })
  }

  get data(): Uint8Array {
    this.atom.reportObserved()
    return this.rawData
  }

  set(index: number, value: number) {
    this.rawData[index] = value
    this.atom.reportChanged()
  }
}
