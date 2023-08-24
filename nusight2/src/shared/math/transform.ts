export type Rotate = number;
export type Scale = { readonly x: number; readonly y: number };
export type Translate = { readonly x: number; readonly y: number };

export type TransformOpts = {
  readonly anticlockwise: boolean;
  readonly rotate: Rotate;
  readonly scale: Scale;
  readonly translate: Translate;
};

export class Transform {
  readonly anticlockwise: boolean;
  readonly rotate: Rotate;
  readonly scale: Scale;
  readonly translate: Translate;

  constructor(opts: TransformOpts) {
    this.anticlockwise = opts.anticlockwise;
    this.rotate = opts.rotate;
    this.scale = opts.scale;
    this.translate = opts.translate;
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
    });
  }

  static translate(x: number, y: number): Transform {
    return Transform.of({ translate: { x, y } });
  }

  then(transform: Transform): Transform {
    const { anticlockwise, rotate, scale, translate } = transform;

    const scaleX = this.scale.x;
    const scaleY = this.scale.y;
    const theta = this.rotate * (this.anticlockwise ? 1 : -1);

    const cosTheta = Math.cos(theta);
    const sinTheta = Math.sin(theta);

    const rotationMatrix = [cosTheta, -sinTheta, sinTheta, cosTheta];

    return new Transform({
      anticlockwise: this.anticlockwise,
      rotate: this.rotate + rotate * (anticlockwise === this.anticlockwise ? 1 : -1),
      scale: {
        x: this.scale.x * scale.x,
        y: this.scale.y * scale.y,
      },
      translate: {
        x: this.translate.x + scaleX * (translate.x * rotationMatrix[0] + translate.y * rotationMatrix[1]),
        y: this.translate.y + scaleY * (translate.x * rotationMatrix[2] + translate.y * rotationMatrix[3]),
      },
    });
  }

  inverse(): Transform {
    const rotate = -this.rotate;
    const scaleX = 1 / this.scale.x;
    const scaleY = 1 / this.scale.y;

    const cosTheta = Math.cos(rotate);
    const sinTheta = Math.sin(rotate);
    const rotationMatrix = [cosTheta, -sinTheta, sinTheta, cosTheta];

    return new Transform({
      anticlockwise: this.anticlockwise,
      rotate: -this.rotate,
      scale: { x: scaleX, y: scaleY },
      translate: {
        x: -(this.translate.x * rotationMatrix[0] * scaleX) - this.translate.y * rotationMatrix[1] * scaleY,
        y: -(this.translate.x * rotationMatrix[2] * scaleX) - this.translate.y * rotationMatrix[3] * scaleY,
      },
    });
  }

  isIdentity(): boolean {
    return (
      this.scale.x === 1 && this.scale.y === 1 && this.translate.x === 0 && this.translate.y === 0 && this.rotate === 0
    );
  }
}
