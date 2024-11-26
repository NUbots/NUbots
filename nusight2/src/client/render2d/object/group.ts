import { observable } from "mobx";

import { Transform } from "../../../shared/math/transform";

import { Geometry } from "./geometry";
import { Shape } from "./shape";

export type GroupOpts = {
  children: (Group | Shape<Geometry>)[];
  transform: Transform;
};

export class Group {
  @observable accessor children: (Group | Shape<Geometry>)[];
  @observable accessor transform: Transform;

  constructor(opts: GroupOpts) {
    this.children = opts.children;
    this.transform = opts.transform;
  }

  static of({ children = [], transform = Transform.of() }: Partial<GroupOpts> = {}): Group {
    return new Group({
      children,
      transform,
    });
  }

  add(obj: Group | Shape<Geometry>): void {
    this.children.push(obj);
  }
}
