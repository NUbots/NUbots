import { observable } from "mobx";

import { FieldDimensions } from "../../../../shared/field/dimensions";

export class GroundModel {
  @observable bottomGoalColor: string;
  @observable dimensions: FieldDimensions;
  @observable fieldColor: string;
  @observable lineColor: string;
  @observable topGoalColor: string;

  constructor({ bottomGoalColor, dimensions, fieldColor, lineColor, topGoalColor }: GroundModel) {
    this.bottomGoalColor = bottomGoalColor;
    this.dimensions = dimensions;
    this.fieldColor = fieldColor;
    this.lineColor = lineColor;
    this.topGoalColor = topGoalColor;
  }

  static of() {
    return new GroundModel({
      bottomGoalColor: "#0000ff",
      dimensions: FieldDimensions.of(),
      fieldColor: "#15803d",
      lineColor: "#ffffff",
      topGoalColor: "#ffff00",
    });
  }
}
