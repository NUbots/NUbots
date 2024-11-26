import { observable } from "mobx";

import { FieldDimensions } from "../../../../../shared/field/dimensions";

export class FieldModel {
  @observable accessor dimensions: FieldDimensions;
  @observable accessor fieldColor: string;
  @observable accessor lineColor: string;
  @observable accessor fieldType: string;

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
      fieldType: "robocup",
    });
  }
}

interface FieldModelOpts {
  dimensions: FieldDimensions;
  fieldColor: string;
  lineColor: string;
  fieldType: string;
}
