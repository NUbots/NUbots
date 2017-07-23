import { observable } from 'mobx'

export class Transform {
  @observable public rotate: number
  @observable public scale: { x: number, y: number }
  @observable public translate: { x: number, y: number }

  public constructor(opts: Transform) {
    this.rotate = opts.rotate
    this.scale = opts.scale
    this.translate = opts.translate
  }

  public static of(opts: Partial<Transform> = {}): Transform {
    return new Transform({
      rotate: opts.rotate || 0,
      scale: opts.scale || { x: 1, y: 1 },
      translate: opts.translate || { x: 0, y: 0 },
    })
  }
}
