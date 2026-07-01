export const popoutBroadcastChannel = new BroadcastChannel("nusight/popout");

export class PopoutParentRendered {
  static type = "popout/parent-rendered";

  /** The name of the popout parent-child pair */
  name: string;

  constructor(opts: { name: string }) {
    this.name = opts.name;
  }
}

export class PopoutChildRendered {
  static type = "popout/child-rendered";

  /** The name of the popout parent-child pair */
  name: string;

  constructor(opts: { name: string }) {
    this.name = opts.name;
  }
}
