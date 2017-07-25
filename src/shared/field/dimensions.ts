import { observable } from 'mobx'

interface FieldModelValues {
  lineWidth: number
  markWidth: number
  fieldLength: number
  fieldWidth: number
  goalDepth: number
  goalWidth: number
  goalAreaLength: number
  goalAreaWidth: number
  goalCrossbarHeight: number
  goalPostDiameter: number
  goalNetHeight: number
  penaltyMarkDistance: number
  centerCircleDiameter: number
  borderStripMinWidth: number
}

export class FieldDimensions {
  @observable public lineWidth: number
  @observable public markWidth: number
  @observable public fieldLength: number
  @observable public fieldWidth: number
  @observable public goalDepth: number
  @observable public goalWidth: number
  @observable public goalAreaLength: number
  @observable public goalAreaWidth: number
  @observable public goalCrossbarHeight: number
  @observable public goalPostDiameter: number
  @observable public goalNetHeight: number
  @observable public penaltyMarkDistance: number
  @observable public centerCircleDiameter: number
  @observable public borderStripMinWidth: number

  constructor(values: FieldModelValues) {
    this.lineWidth = values.lineWidth
    this.markWidth = values.markWidth
    this.fieldLength = values.fieldLength
    this.fieldWidth = values.fieldWidth
    this.goalDepth = values.goalDepth
    this.goalWidth = values.goalWidth
    this.goalAreaLength = values.goalAreaLength
    this.goalAreaWidth = values.goalAreaWidth
    this.goalCrossbarHeight = values.goalCrossbarHeight
    this.goalPostDiameter = values.goalPostDiameter
    this.goalNetHeight = values.goalNetHeight
    this.penaltyMarkDistance = values.penaltyMarkDistance
    this.centerCircleDiameter = values.centerCircleDiameter
    this.borderStripMinWidth = values.borderStripMinWidth
  }

  public static preYear2014() {
    return new FieldDimensions({
      lineWidth: 0.05,
      markWidth: 0.1,
      fieldLength: 6,
      fieldWidth: 4,
      goalDepth: 0.5,
      goalWidth: 1.5,
      goalAreaLength: 0.6,
      goalAreaWidth: 2.2,
      goalCrossbarHeight: 1.1,
      goalPostDiameter: 0.1,
      goalNetHeight: 0.8,
      penaltyMarkDistance: 1.80,
      centerCircleDiameter: 1.20,
      borderStripMinWidth: 0.7,
    })
  }

  public static postYear2017() {
    return new FieldDimensions({
      lineWidth: 0.06,
      markWidth: 0.1,
      fieldLength: 9,
      fieldWidth: 6,
      goalDepth: 0.6,
      goalWidth: 2.6,
      goalAreaLength: 1,
      goalAreaWidth: 5,
      goalCrossbarHeight: 1.8,
      goalPostDiameter: 0.1,
      goalNetHeight: 1,
      penaltyMarkDistance: 2.05,
      centerCircleDiameter: 1.56,
      borderStripMinWidth: 0.7,
    })
  }
}
