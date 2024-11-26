import { Property } from "csstype";
import { observable } from "mobx";

type Fill = { color: string; alpha: number };
type Stroke = { color: string; alpha: number; width: number; nonScaling: boolean };

export type BasicAppearanceOpts = {
  fill?: Partial<Fill>;
  stroke?: Partial<Stroke>;
  cursor?: Property.Cursor;
};

export class BasicAppearance {
  @observable accessor fill?: Fill;
  @observable accessor stroke?: Stroke;
  @observable accessor cursor?: Property.Cursor;
  @observable accessor nonScaling?: boolean;

  constructor({ fill, stroke, cursor }: { fill?: Fill; stroke?: Stroke; cursor?: Property.Cursor }) {
    this.fill = fill;
    this.stroke = stroke;
    this.cursor = cursor;
  }

  static of({ fill, stroke, cursor }: BasicAppearanceOpts = {}): BasicAppearance {
    return new BasicAppearance({
      fill: fill && { color: "#000000", alpha: 1, ...fill },
      stroke: stroke && { color: "#000000", alpha: 1, width: 1, nonScaling: false, ...stroke },
      cursor,
    });
  }
}
