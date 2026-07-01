import { observable } from "mobx";

interface FieldModelValues {
  lineWidth: number;
  markWidth: number;
  fieldLength: number;
  fieldWidth: number;
  goalDepth: number;
  goalWidth: number;
  goalAreaLength: number;
  goalAreaWidth: number;
  penaltyAreaLength: number;
  penaltyAreaWidth: number;
  goalCrossbarHeight: number;
  goalPostDiameter: number;
  goalNetHeight: number;
  penaltyMarkDistance: number;
  centerCircleDiameter: number;
  borderStripMinWidth: number;
}

export class FieldDimensions {
  @observable lineWidth: number;
  @observable markWidth: number;
  @observable fieldLength: number;
  @observable fieldWidth: number;
  @observable goalDepth: number;
  @observable goalWidth: number;
  @observable goalAreaLength: number;
  @observable goalAreaWidth: number;
  @observable penaltyAreaLength: number;
  @observable penaltyAreaWidth: number;
  @observable goalCrossbarHeight: number;
  @observable goalPostDiameter: number;
  @observable goalNetHeight: number;
  @observable penaltyMarkDistance: number;
  @observable centerCircleDiameter: number;
  @observable borderStripMinWidth: number;

  constructor(values: FieldModelValues) {
    this.lineWidth = values.lineWidth;
    this.markWidth = values.markWidth;
    this.fieldLength = values.fieldLength;
    this.fieldWidth = values.fieldWidth;
    this.goalDepth = values.goalDepth;
    this.goalWidth = values.goalWidth;
    this.goalAreaLength = values.goalAreaLength;
    this.goalAreaWidth = values.goalAreaWidth;
    this.penaltyAreaLength = values.penaltyAreaLength;
    this.penaltyAreaWidth = values.penaltyAreaWidth;
    this.goalCrossbarHeight = values.goalCrossbarHeight;
    this.goalPostDiameter = values.goalPostDiameter;
    this.goalNetHeight = values.goalNetHeight;
    this.penaltyMarkDistance = values.penaltyMarkDistance;
    this.centerCircleDiameter = values.centerCircleDiameter;
    this.borderStripMinWidth = values.borderStripMinWidth;
  }
  static of() {
    return new FieldDimensions({
      lineWidth: 0.06,
      markWidth: 0.1,
      fieldLength: 9.0,
      fieldWidth: 6.0,
      goalDepth: 0.5,
      goalWidth: 1.8,
      goalAreaLength: 0.9,
      goalAreaWidth: 2.9,
      penaltyAreaLength: 1.9,
      penaltyAreaWidth: 3.9,
      goalCrossbarHeight: 1.2,
      goalPostDiameter: 0.1,
      goalNetHeight: 1.2,
      penaltyMarkDistance: 1.47,
      centerCircleDiameter: 1.5,
      borderStripMinWidth: 1.0,
    });
  }
}
