import { observable } from 'mobx'

import { Transform } from '../../math/transform'

import { Object2d } from './object2d'

export type GroupOpts = {
  children: Object2d[]
  transform: Transform
}

export class Group implements Object2d {
  @observable children: Object2d[]
  @observable transform: Transform

  constructor(opts: GroupOpts) {
    this.children = opts.children
    this.transform = opts.transform
  }

  static of({
    children = [],
    transform = Transform.of(),
  }: Partial<GroupOpts> = {}): Group {
    return new Group({
      children,
      transform,
    })
  }

  add(obj: Object2d): void {
    this.children.push(obj)
  }
}
