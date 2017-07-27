import { observable } from 'mobx'
import { FieldDimensions } from '../../../../shared/field/dimensions'

export class GroundModel {
  @observable public bottomGoalColor: string
  @observable public dimensions: FieldDimensions
  @observable public fieldColor: string
  @observable public lineColor: string
  @observable public topGoalColor: string

  public constructor(opts: GroundModel) {
    Object.assign(this, opts)
  }

  public static of() {
    return new GroundModel({
      bottomGoalColor: 'blue',
      dimensions: FieldDimensions.postYear2017(),
      fieldColor: '#009688',
      lineColor: '#fff',
      topGoalColor: 'yellow',
    })
  }
}
