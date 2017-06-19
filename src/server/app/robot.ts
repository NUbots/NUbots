interface RobotOpts {
  name: string
}

export class Robot {
  public name: string

  public constructor(opts: RobotOpts) {
    this.name = opts.name
  }

  public static of(opts: RobotOpts) {
    return new Robot(opts)
  }
}
