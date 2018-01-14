import { observable } from 'mobx'

export type Rotate = number
export type Scale = { x: number, y: number }
export type Translate = { x: number, y: number }

export type TransformOpts = {
  anticlockwise: boolean
  rotate: Rotate
  scale: Scale
  translate: Translate
}

export class Transform {
  @observable anticlockwise: boolean
  @observable rotate: Rotate
  @observable scale: Scale
  @observable translate: Translate

  constructor(opts: TransformOpts) {
    this.anticlockwise = opts.anticlockwise
    this.rotate = opts.rotate
    this.scale = opts.scale
    this.translate = opts.translate
  }

  static of({
              anticlockwise = true,
              rotate = 0,
              scale = { x: 1, y: 1 },
              translate = { x: 0, y: 0 },
            }: Partial<Transform> = {}): Transform {
    return new Transform({
      anticlockwise,
      rotate,
      scale,
      translate,
    })
  }

  then(transform: Transform): Transform {
    const { anticlockwise, rotate, scale, translate } = transform

    const scaleX = this.scale.x
    const scaleY = this.scale.y
    const theta = this.rotate * (this.anticlockwise ? 1 : -1)

    const cosTheta = Math.cos(theta)
    const sinTheta = Math.sin(theta)

    const rotationMatrix = [
      cosTheta, -sinTheta,
      sinTheta, cosTheta,
    ]

    this.scale.x *= scale.x
    this.scale.y *= scale.y
    this.rotate += rotate * (anticlockwise === this.anticlockwise ? 1 : -1)

    const x = translate.x
    const y = translate.y

    this.translate.x += scaleX * (x * rotationMatrix[0] + y * rotationMatrix[1])
    this.translate.y += scaleY * (x * rotationMatrix[2] + y * rotationMatrix[3])

    return this
  }

  inverse(): Transform {
    return new Transform({
      anticlockwise: this.anticlockwise,
      scale: { x: 1 / this.scale.x, y: 1 / this.scale.y },
      rotate: -this.rotate,
      translate: { x: -this.translate.x, y: -this.translate.y },
    })
  }

  clone(): Transform {
    return new Transform({
      anticlockwise: this.anticlockwise,
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

  setTranslate(x: number, y: number): Transform {
    this.translate.x = x
    this.translate.y = y
    return this
  }
}
