import { action } from 'mobx'
import { observable } from 'mobx'
import { computed } from 'mobx'

export type Rotate = number
export type Scale = { x: number, y: number }
export type Translate = { x: number, y: number }

export type TransformOpts = {
  rotate: Rotate
  scale: Scale
  translate: Translate
}

export class Transform {
  @observable public rotate: Rotate
  @observable public scale: Scale
  @observable public translate: Translate

  public constructor(opts: TransformOpts) {
    this.rotate = opts.rotate
    this.scale = opts.scale
    this.translate = opts.translate
  }

  public static of({
    rotate = 0,
    scale = { x: 1, y: 1 },
    translate = { x: 0, y: 0 },
  }: Partial<Transform> = {}): Transform {
    return new Transform({
      rotate,
      scale,
      translate,
    })
  }

  @action
  public then(transform: Transform): Transform {
    const { rotate, scale, translate } = transform

    const scaleX = this.scale.x
    const scaleY = this.scale.y
    const theta = this.rotate

    const cosTheta = Math.cos(theta)
    const sinTheta = Math.sin(theta)

    const rotationMatrix = [
      cosTheta, -sinTheta,
      sinTheta, cosTheta,
    ]

    this.scale.x *= scale.x
    this.scale.y *= scale.y
    this.rotate += rotate

    const x = translate.x
    const y = translate.y

    this.translate.x += scaleX * (x * rotationMatrix[0] + y * rotationMatrix[1])
    this.translate.y += scaleY * (x * rotationMatrix[2] + y * rotationMatrix[3])

    return this
  }

  @computed
  public get inverse(): Transform {
    return new Transform({
      scale: { x: 1 / this.scale.x, y: 1 / this.scale.y },
      rotate: -this.rotate,
      translate: { x: this.translate.x, y: this.translate.y },
    })
  }

  public clone(): Transform {
    return new Transform({
      rotate: this.rotate,
      scale: {
        x: this.scale.x,
        y: this.scale.y,
      },
      translate: {
        x: this.translate.x,
        y: this.translate.y,
      },
    })
  }

  @action
  public setTranslate(x: number, y: number): Transform {
    this.translate.x = x
    this.translate.y = y
    return this
  }
}
