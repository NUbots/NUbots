import { observable } from "mobx";

import { Appearance } from "../appearance/appearance";
import { BasicAppearance } from "../appearance/basic_appearance";
import { Render2DEventHandlers } from "../event/event_handlers";

import { Geometry } from "./geometry";

export type ShapeOpts<T extends Geometry> = {
  geometry: T;
  appearance?: Appearance;
  eventHandlers?: Render2DEventHandlers;
};

export class Shape<T extends Geometry> {
  @observable accessor geometry: T;
  @observable accessor appearance: Appearance;
  @observable accessor eventHandlers: Render2DEventHandlers;

  constructor(opts: Required<ShapeOpts<T>>) {
    this.geometry = opts.geometry;
    this.appearance = opts.appearance;
    this.eventHandlers = opts.eventHandlers;
  }

  static of<T extends Geometry>({ geometry, appearance = BasicAppearance.of(), eventHandlers = {} }: ShapeOpts<T>) {
    return new Shape<T>({ geometry, appearance, eventHandlers });
  }
}
