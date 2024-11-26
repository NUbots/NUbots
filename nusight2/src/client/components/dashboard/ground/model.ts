import { observable } from "mobx";

import { FieldDimensions } from "../../../../shared/field/dimensions";

export class GroundModel {
  @observable accessor bottomGoalColor: string;
  @observable accessor dimensions: FieldDimensions;
  @observable accessor fieldColor: string;
  @observable accessor lineColor: string;
  @observable accessor topGoalColor: string;

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
