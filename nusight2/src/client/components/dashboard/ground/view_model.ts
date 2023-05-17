import { computed } from "mobx";
import { createTransformer } from "mobx-utils";

import { Vector2 } from "../../../../shared/math/vector2";
import { BasicAppearance } from "../../../render2d/appearance/basic_appearance";
import { LineAppearance } from "../../../render2d/appearance/line_appearance";
import { CircleGeometry } from "../../../render2d/geometry/circle_geometry";
import { LineGeometry } from "../../../render2d/geometry/line_geometry";
import { PolygonGeometry } from "../../../render2d/geometry/polygon_geometry";
import { Group } from "../../../render2d/object/group";
import { Shape } from "../../../render2d/object/shape";

import { GroundModel } from "./model";

export class GroundViewModel {
  constructor(private model: GroundModel) {}

  static of = createTransformer((model: GroundModel): GroundViewModel => {
    return new GroundViewModel(model);
  });

  @computed
  get ground(): Group {
    return Group.of({
      children: [this.grass, this.goals, this.fieldLines],
    });
  }

  @computed
  private get grass() {
    const borderStripMinWidth = this.model.dimensions.borderStripMinWidth;
    const goalDepth = this.model.dimensions.goalDepth;
    const height = this.model.dimensions.fieldLength + borderStripMinWidth * 2 + goalDepth * 2;
    const width = this.model.dimensions.fieldWidth + borderStripMinWidth * 2;
    const x = -this.model.dimensions.fieldLength * 0.5 - goalDepth - borderStripMinWidth;
    const y = -this.model.dimensions.fieldWidth * 0.5 - borderStripMinWidth;
    return Shape.of({
      geometry: this.getRectanglePolygon({ x, y, width, height }),
      appearance: BasicAppearance.of({
        fill: { color: this.model.fieldColor },
      }),
    });
  }

  @computed
  private get goals() {
    const dimensions = this.model.dimensions;
    const width = dimensions.goalWidth;
    const height = dimensions.goalDepth;
    const y = -width * 0.5;
    const goal = (x: number, strokeColor: string) =>
      Shape.of({
        geometry: this.getRectanglePolygon({ x, y, width, height }),
        appearance: BasicAppearance.of({
          stroke: { width: dimensions.lineWidth, color: strokeColor },
        }),
      });
    return Group.of({
      children: [
        goal(dimensions.fieldLength * 0.5, this.model.topGoalColor),
        goal(-dimensions.fieldLength * 0.5 - height, this.model.bottomGoalColor),
      ],
    });
  }

  @computed
  private get fieldLines() {
    return Group.of({
      children: [
        this.centerCircle,
        this.centerMark,
        this.halfwayLine,
        this.fieldBorder,
        this.goalAreas,
        this.penaltyMarkers,
      ],
    });
  }

  @computed
  private get centerCircle() {
    return Shape.of({
      geometry: CircleGeometry.of({ radius: this.model.dimensions.centerCircleDiameter * 0.5 }),
      appearance: BasicAppearance.of({
        stroke: { width: this.model.dimensions.lineWidth, color: this.model.lineColor },
      }),
    });
  }

  @computed
  private get centerMark() {
    const width = this.model.dimensions.lineWidth * 2;
    const color = this.model.lineColor;
    return Group.of({
      children: [
        Shape.of({
          geometry: LineGeometry.of({
            origin: Vector2.of(0, width),
            target: Vector2.of(0, -width),
          }),
          appearance: LineAppearance.of({ stroke: { width, color } }),
        }),
        Shape.of({
          geometry: LineGeometry.of({
            origin: Vector2.of(width, 0),
            target: Vector2.of(-width, 0),
          }),
          appearance: LineAppearance.of({ stroke: { width, color } }),
        }),
      ],
    });
  }

  @computed
  private get halfwayLine() {
    return Shape.of({
      geometry: LineGeometry.of({
        origin: Vector2.of(0, this.model.dimensions.fieldWidth * 0.5),
        target: Vector2.of(0, -this.model.dimensions.fieldWidth * 0.5),
      }),
      appearance: LineAppearance.of({
        stroke: {
          width: this.model.dimensions.lineWidth,
          color: this.model.lineColor,
        },
      }),
    });
  }

  @computed
  private get fieldBorder() {
    return Shape.of({
      geometry: this.getRectanglePolygon({
        x: -this.model.dimensions.fieldLength * 0.5,
        y: -this.model.dimensions.fieldWidth * 0.5,
        width: this.model.dimensions.fieldWidth,
        height: this.model.dimensions.fieldLength,
      }),
      appearance: BasicAppearance.of({
        stroke: {
          width: this.model.dimensions.lineWidth,
          color: this.model.lineColor,
        },
      }),
    });
  }

  @computed
  private get goalAreas() {
    const fieldLength = this.model.dimensions.fieldLength;
    const height = this.model.dimensions.goalAreaLength;
    const width = this.model.dimensions.goalAreaWidth;
    const y = -width * 0.5;
    const goalArea = (x: number) =>
      Shape.of({
        geometry: this.getRectanglePolygon({ x, y, width, height }),
        appearance: BasicAppearance.of({
          stroke: {
            width: this.model.dimensions.lineWidth,
            color: this.model.lineColor,
          },
        }),
      });
    return Group.of({
      children: [goalArea(fieldLength * 0.5 - height), goalArea(-fieldLength * 0.5)],
    });
  }

  @computed
  private get penaltyMarkers() {
    const fieldLength = this.model.dimensions.fieldLength;
    const penaltyMarkDistance = this.model.dimensions.penaltyMarkDistance;
    const width = this.model.dimensions.lineWidth;
    const marker = (x: number) =>
      Group.of({
        children: [
          Shape.of({
            geometry: LineGeometry.of({
              origin: Vector2.of(x + width, 0),
              target: Vector2.of(x - width, 0),
            }),
            appearance: LineAppearance.of({
              stroke: {
                width,
                color: this.model.lineColor,
              },
            }),
          }),
          Shape.of({
            geometry: LineGeometry.of({
              origin: Vector2.of(x, width),
              target: Vector2.of(x, -width),
            }),
            appearance: LineAppearance.of({
              stroke: {
                width,
                color: this.model.lineColor,
              },
            }),
          }),
        ],
      });
    return Group.of({
      children: [marker(fieldLength * 0.5 - penaltyMarkDistance), marker(-(fieldLength * 0.5) + penaltyMarkDistance)],
    });
  }

  private getRectanglePolygon(opts: { x: number; y: number; width: number; height: number }): PolygonGeometry {
    // Width is defined along the positive y axis (across the field), and height along the positive x axis (along the
    // field). This matches the field definitions model.
    return PolygonGeometry.of([
      Vector2.of(opts.x, opts.y), // Bottom right
      Vector2.of(opts.x, opts.y + opts.width), // Bottom left
      Vector2.of(opts.x + opts.height, opts.y + opts.width), // Top left
      Vector2.of(opts.x + opts.height, opts.y), // Top right
    ]);
  }
}
