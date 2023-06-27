import { observable } from "mobx";

import { FieldDimensions } from "../../../../shared/field/dimensions";

export class FieldModel {
  @observable dimensions: FieldDimensions;
  @observable fieldColor: string;
  @observable lineColor: string;

  constructor({ dimensions, fieldColor, lineColor }: FieldModelOpts) {
    this.dimensions = dimensions;
    this.fieldColor = fieldColor;
    this.lineColor = lineColor;
  }

  static of() {
    return new FieldModel({
      dimensions: FieldDimensions.postYear2017(),
      fieldColor: "#00cc00",
      lineColor: "#ffffff",
    });
  }
}

interface FieldModelOpts {
  dimensions: FieldDimensions;
  fieldColor: string;
  lineColor: string;
}
