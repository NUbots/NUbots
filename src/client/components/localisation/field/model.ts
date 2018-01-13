import { observable } from 'mobx'

import { FieldDimensions } from '../../../../shared/field/dimensions'

export class FieldModel {
  @observable public dimensions: FieldDimensions
  @observable public fieldColor: string
  @observable public lineColor: string

  public constructor({ dimensions, fieldColor, lineColor }: FieldModelOpts) {
    this.dimensions = dimensions
    this.fieldColor = fieldColor
    this.lineColor = lineColor
  }

  public static of() {
    return new FieldModel({
      dimensions: FieldDimensions.postYear2017(),
      fieldColor: '#009900',
      lineColor: '#ffffff',
    })
  }
}

interface FieldModelOpts {
  dimensions: FieldDimensions
  fieldColor: string
  lineColor: string
}
