import { observable } from "mobx";

import { FieldDimensions } from "../../../../../shared/field/dimensions";

export class FieldModel {
  @observable dimensions: FieldDimensions;
  @observable fieldColor: string;
  @observable lineColor: string;
  @observable fieldType: string;
  @observable blueGoalColor: string;
  @observable yellowGoalColor: string;

  constructor({ dimensions, fieldColor, lineColor, fieldType, blueGoalColor, yellowGoalColor }: FieldModelOpts) {
    this.dimensions = dimensions;
    this.fieldColor = fieldColor;
    this.lineColor = lineColor;
    this.fieldType = fieldType;
    this.blueGoalColor = blueGoalColor;
    this.yellowGoalColor = yellowGoalColor;
  }

  static of() {
    return new FieldModel({
      dimensions: FieldDimensions.of(),
      fieldColor: "#00cc00",
      lineColor: "#ffffff",
      fieldType: "robocup",
      blueGoalColor: "#0000ff",
      yellowGoalColor: "#ffff00",
    });
  }
}

interface FieldModelOpts {
  dimensions: FieldDimensions;
  fieldColor: string;
  lineColor: string;
  fieldType: string;
  blueGoalColor: string;
  yellowGoalColor: string;
}
