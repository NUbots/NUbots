import { action } from 'mobx'
import { observable } from 'mobx'
import { Transform } from '../../math/transform'
import { Object2d } from './object2d'

export type GroupOpts = {
  children: Object2d[]
  transform: Transform
}

export class Group implements Object2d {
  @observable public children: Object2d[]
  @observable public transform: Transform

  public constructor(opts: GroupOpts) {
    this.children = opts.children
    this.transform = opts.transform
  }

  public static of({
    children = [],
    transform = Transform.of(),
  }: Partial<GroupOpts> = {}): Group {
    return new Group({
      children,
      transform,
    })
  }

  @action
  public add(obj: Object2d): void {
    this.children.push(obj)
  }
}
