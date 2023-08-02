import React from "react";
import { computed } from "mobx";
import { observer } from "mobx-react";
import * as THREE from "three";
import { Matrix4 } from "three";
import * as BufferGeometryUtils from "three/examples/jsm/utils/BufferGeometryUtils";

import { FieldModel } from "./model";

@observer
export class FieldView extends React.Component<{
  model: FieldModel;
}> {
  private get model() {
    return this.props.model;
  }

  render() {
    const dim = this.model.dimensions;
    return (
      <object3D>
        <mesh>
          <planeBufferGeometry
            args={[
              dim.fieldLength + dim.goalDepth * 2 + dim.borderStripMinWidth * 2,
              dim.fieldWidth + dim.borderStripMinWidth * 2,
            ]}
          />
          <meshBasicMaterial color={this.model.fieldColor} />
        </mesh>
        <mesh geometry={this.fieldLinesGeometry} position={[0, 0, 0.001]}>
          <meshBasicMaterial color={this.model.lineColor} />
        </mesh>
      </object3D>
    );
  }

  @computed
  private get fieldLinesGeometry() {
    const centerCircle = this.centerCircle;

    const fieldWidth = this.model.dimensions.fieldWidth;
    const fieldLength = this.model.dimensions.fieldLength;
    const lineWidth = this.model.dimensions.lineWidth;
    const goalAreaWidth = this.model.dimensions.goalAreaWidth;
    const goalAreaLength = this.model.dimensions.goalAreaLength;
    const penaltyAreaWidth = this.model.dimensions.penaltyAreaWidth;
    const penaltyAreaLength = this.model.dimensions.penaltyAreaLength;
    const goalDepth = this.model.dimensions.goalDepth;
    const goalWidth = this.model.dimensions.goalWidth;
    const penaltyMarkDistance = this.model.dimensions.penaltyMarkDistance;

    const halfLength = fieldLength * 0.5;
    const halfWidth = fieldWidth * 0.5;
    const halfGoalAreaWidth = goalAreaWidth * 0.5;
    const halfPenaltyAreaWidth = penaltyAreaWidth * 0.5;
    const halfGoalWidth = goalWidth * 0.5;

    const blueHalf = this.buildRectangle(-halfLength, -halfWidth, halfLength, fieldWidth, lineWidth);
    const blueHalfPenaltyArea = this.buildRectangle(
      -halfLength,
      -halfPenaltyAreaWidth,
      penaltyAreaLength,
      penaltyAreaWidth,
      lineWidth,
    );
    const blueHalfGoalArea = this.buildRectangle(
      -halfLength,
      -halfGoalAreaWidth,
      goalAreaLength,
      goalAreaWidth,
      lineWidth,
    );
    const blueHalfGoal = this.buildRectangle(-halfLength - goalDepth, -halfGoalWidth, goalDepth, goalWidth, lineWidth);
    const blueHalfPenaltyMark = this.buildRectangle(-halfLength + penaltyMarkDistance, 0, 0, 0, lineWidth);

    const yellowHalf = this.buildRectangle(0, -halfWidth, halfLength, fieldWidth, lineWidth);
    const yellowHalfPenaltyArea = this.buildRectangle(
      halfLength - penaltyAreaLength,
      -halfPenaltyAreaWidth,
      penaltyAreaLength,
      penaltyAreaWidth,
      lineWidth,
    );
    const yellowHalfGoalArea = this.buildRectangle(
      halfLength - goalAreaLength,
      -halfGoalAreaWidth,
      goalAreaLength,
      goalAreaWidth,
      lineWidth,
    );
    const yellowHalfGoal = this.buildRectangle(halfLength, -halfGoalWidth, goalDepth, goalWidth, lineWidth);
    const yellowHalfPenaltyMark = this.buildRectangle(halfLength - penaltyMarkDistance, 0, 0, 0, lineWidth);

    return BufferGeometryUtils.mergeBufferGeometries([
      centerCircle,
      blueHalf,
      blueHalfPenaltyArea,
      blueHalfGoalArea,
      blueHalfGoal,
      blueHalfPenaltyMark,
      yellowHalf,
      yellowHalfPenaltyArea,
      yellowHalfGoalArea,
      yellowHalfGoal,
      yellowHalfPenaltyMark,
    ]);
  }

  @computed
  private get centerCircle() {
    return new THREE.RingBufferGeometry(
      (this.model.dimensions.centerCircleDiameter - this.model.dimensions.lineWidth) * 0.5,
      (this.model.dimensions.centerCircleDiameter + this.model.dimensions.lineWidth) * 0.5,
      128,
    );
  }

  private buildRectangle(x: number, y: number, w: number, h: number, lw: number) {
    const x1 = x - lw * 0.5;
    const x2 = x + w + lw * 0.5;

    const topLine = this.buildHorizontalLine(x1, x2, y, lw);
    const bottomLine = this.buildHorizontalLine(x1, x2, y + h, lw);

    const leftLine = this.buildVerticalLine(y, y + h, x, lw);
    const rightLine = this.buildVerticalLine(y, y + h, x + w, lw);

    return BufferGeometryUtils.mergeBufferGeometries([topLine, bottomLine, leftLine, rightLine]);
  }

  private buildHorizontalLine(x1: number, x2: number, y: number, width: number) {
    const length = x2 - x1;
    const hLine = new THREE.PlaneGeometry(length, width);
    hLine.applyMatrix4(new Matrix4().makeTranslation(x1 + length * 0.5, y, 0));
    return hLine;
  }

  private buildVerticalLine(y1: number, y2: number, x: number, width: number) {
    const length = y2 - y1;
    const vLine = new THREE.PlaneGeometry(width, length);
    vLine.applyMatrix4(new Matrix4().makeTranslation(x, y1 + length * 0.5, 0));
    return vLine;
  }
}
