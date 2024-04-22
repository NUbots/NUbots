import { observable } from "mobx";

import { FieldDimensions } from "../../../../shared/field/dimensions";

export class FieldModel {
  @observable dimensions: FieldDimensions;
  @observable fieldColor: string;
  @observable lineColor: string;
  @observable fieldType: string;

  constructor({ dimensions, fieldColor, lineColor, fieldType }: FieldModelOpts) {
    this.dimensions = dimensions;
    this.fieldColor = fieldColor;
    this.lineColor = lineColor;
    this.fieldType = fieldType;
  }

  static of() {
    return new FieldModel({
      dimensions: FieldDimensions.of(),
      fieldColor: "#00cc00",
      lineColor: "#ffffff",
      fieldType: "Robocup",
    });
  }
}

interface FieldModelOpts {
  dimensions: FieldDimensions;
  fieldColor: string;
  lineColor: string;
  fieldType: string;
}
