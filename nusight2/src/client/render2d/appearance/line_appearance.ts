import { observable } from "mobx";

type Stroke = {
  color: string;
  alpha: number;
  width: number;
  cap: "butt" | "round" | "square";
  dashOffset: number;
  join: "bevel" | "round" | "miter";
  nonScaling: boolean;
};

export type LineAppearanceOpts = {
  stroke: Partial<Stroke>;
};

export class LineAppearance {
  @observable stroke: Stroke;

  constructor({ stroke }: { stroke: Stroke }) {
    this.stroke = stroke;
  }

  static of({ stroke }: LineAppearanceOpts = { stroke: {} }): LineAppearance {
    return new LineAppearance({
      stroke: {
        color: "#000000",
        alpha: 1,
        width: 1,
        cap: "butt",
        dashOffset: 0,
        join: "miter",
        nonScaling: false,
        ...stroke,
      },
    });
  }
}
