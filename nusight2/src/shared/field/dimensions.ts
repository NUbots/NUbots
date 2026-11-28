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
      goalDepth: 0.58,
      goalWidth: 2.575,
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

/** The field presets selectable in the localisation view, by field type. */
export const FIELD_PRESETS: Record<string, FieldModelValues> = {
  lab: {
    lineWidth: 0.05,
    markWidth: 0.1,
    fieldLength: 6.8,
    fieldWidth: 5.0,
    goalDepth: 0.4,
    goalWidth: 1.95,
    goalAreaLength: 1.05,
    goalAreaWidth: 2.62,
    penaltyAreaLength: 1.55,
    penaltyAreaWidth: 4.05,
    goalCrossbarHeight: 0.55,
    goalPostDiameter: 0.1,
    goalNetHeight: 1.0,
    penaltyMarkDistance: 1.27,
    centerCircleDiameter: 1.5,
    borderStripMinWidth: 0.38,
  },
  robocup_small: {
    lineWidth: 0.06,
    markWidth: 0.1,
    fieldLength: 9.0,
    fieldWidth: 6.0,
    goalDepth: 0.58,
    goalWidth: 2.575,
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
  },
  // rUNSWift lab field. Goal dimensions not supplied, so taken from robocup_small
  runswift: {
    lineWidth: 0.06,
    markWidth: 0.1,
    fieldLength: 9.0,
    fieldWidth: 6.0,
    goalDepth: 0.58,
    goalWidth: 2.575,
    goalAreaLength: 0.6,
    goalAreaWidth: 2.2,
    penaltyAreaLength: 1.65,
    penaltyAreaWidth: 4.0,
    goalCrossbarHeight: 1.2,
    goalPostDiameter: 0.1,
    goalNetHeight: 1.2,
    penaltyMarkDistance: 1.5,
    centerCircleDiameter: 1.5,
    borderStripMinWidth: 1.0,
  },
  // RoboCup 2026 Humanoid Soccer League M-Field (Middle Division). The rules give ranges for the
  // goal; these match NUSim's M-Field scene and the middle preset in FieldDescription.yaml.
  robocup_middle: {
    lineWidth: 0.05,
    markWidth: 0.1,
    fieldLength: 14.0,
    fieldWidth: 9.0,
    goalDepth: 0.7,
    goalWidth: 2.5,
    goalAreaLength: 1.0,
    goalAreaWidth: 4.0,
    penaltyAreaLength: 3.0,
    penaltyAreaWidth: 6.0,
    goalCrossbarHeight: 1.75,
    goalPostDiameter: 0.1,
    goalNetHeight: 1.8,
    penaltyMarkDistance: 2.0,
    centerCircleDiameter: 3.0,
    borderStripMinWidth: 1.0,
  },
  robocup_large: {
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
  },
  robocup_5v5: {
    lineWidth: 0.06,
    markWidth: 0.1,
    fieldLength: 22.0,
    fieldWidth: 14.0,
    goalDepth: 0.6,
    goalWidth: 2.6,
    goalAreaLength: 2.0,
    goalAreaWidth: 5.0,
    penaltyAreaLength: 5.0,
    penaltyAreaWidth: 8.0,
    goalCrossbarHeight: 1.25,
    goalPostDiameter: 0.1,
    goalNetHeight: 1.2,
    penaltyMarkDistance: 3.5,
    centerCircleDiameter: 4,
    borderStripMinWidth: 1.0,
  },
};
