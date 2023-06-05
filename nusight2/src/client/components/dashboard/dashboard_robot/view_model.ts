import { computed } from "mobx";
import { createTransformer } from "mobx-utils";

import { Transform } from "../../../../shared/math/transform";
import { Vector2 } from "../../../../shared/math/vector2";
import { BasicAppearance } from "../../../render2d/appearance/basic_appearance";
import { LineAppearance } from "../../../render2d/appearance/line_appearance";
import { ArcGeometry } from "../../../render2d/geometry/arc_geometry";
import { ArrowGeometry } from "../../../render2d/geometry/arrow_geometry";
import { CircleGeometry } from "../../../render2d/geometry/circle_geometry";
import { LineGeometry } from "../../../render2d/geometry/line_geometry";
import { MarkerGeometry } from "../../../render2d/geometry/marker_geometry";
import { TextGeometry } from "../../../render2d/geometry/text_geometry";
import { Group } from "../../../render2d/object/group";
import { Shape } from "../../../render2d/object/shape";

import { DashboardRobotModel } from "./model";

export class DashboardRobotViewModel {
  constructor(private model: DashboardRobotModel) {}

  static of = createTransformer((model: DashboardRobotModel): DashboardRobotViewModel => {
    return new DashboardRobotViewModel(model);
  });

  @computed
  get robot(): Group {
    return Group.of({
      children: [this.fieldSpaceGroup, this.robotSpaceGroup],
    });
  }

  @computed
  get fieldSpaceGroup() {
    return Group.of({
      children: [this.ballSight, this.kickTarget, this.ball],
    });
  }

  @computed
  get robotSpaceGroup() {
    return Group.of({
      children: [this.walkCommand, this.robotMarker],
      transform: Transform.of({
        rotate: this.model.robotPosition.z,
        translate: {
          x: this.model.robotPosition.x,
          y: this.model.robotPosition.y,
        },
      }),
    });
  }

  @computed
  private get walkCommand() {
    const t = 2;
    const translation = Vector2.from(this.model.walkCommand);
    const rotation = this.model.walkCommand.z;
    const radius = translation.length / (Math.abs(rotation) + 1e-10);
    const origin = Vector2.of(-translation.y, translation.x).divideScalar(rotation);
    const arcLength = rotation * t;
    const angle = Math.atan2(translation.y / rotation, translation.x / rotation) - Math.PI / 2;

    const startAngle = angle;
    const endAngle = startAngle + arcLength;

    return Shape.of({
      geometry: ArcGeometry.of({
        origin,
        radius,
        startAngle,
        endAngle,
        anticlockwise: rotation > 0,
      }),
      appearance: BasicAppearance.of({
        stroke: { width: 0.025, color: "#000000" },
      }),
    });
  }

  @computed
  private get ball() {
    return Shape.of({
      geometry: CircleGeometry.of({
        radius: 0.1,
        x: this.model.ballPosition.x,
        y: this.model.ballPosition.y,
      }),
      appearance: BasicAppearance.of({
        fill: { color: this.model.ballColor },
      }),
    });
  }

  @computed
  private get ballSight() {
    return Shape.of({
      geometry: LineGeometry.of({
        origin: Vector2.from(this.model.robotPosition),
        target: this.model.ballPosition,
      }),
      appearance: LineAppearance.of({
        stroke: { width: 0.025, color: this.model.ballSightColor },
      }),
    });
  }

  @computed
  private get kickTarget() {
    const origin = this.model.ballPosition;
    const difference = this.model.kickTarget.subtract(origin);
    return Shape.of({
      geometry: ArrowGeometry.of({
        direction: difference.normalize(),
        headLength: 0.3,
        headWidth: 0.15,
        length: difference.length,
        origin,
        width: 0.025,
      }),
      appearance: BasicAppearance.of({
        fill: { color: this.model.kickTargetColor },
      }),
    });
  }

  @computed
  private get robotMarker() {
    const radius = 0.15;
    return Group.of({
      children: [
        Shape.of({
          geometry: MarkerGeometry.of({
            radius,
            x: 0,
            y: 0,
          }),
          appearance: BasicAppearance.of({
            fill: { color: this.model.robotColor },
          }),
        }),
        Shape.of({
          geometry: TextGeometry.of({
            text: this.model.playerId.toString(),
            worldAlignment: true,
            textAlign: "middle",
            textBaseline: "middle",
            fontSize: `${radius * 1.9}px`,
            x: 0,
            y: 0,
          }),
          appearance: BasicAppearance.of({
            fill: { color: this.model.textColor },
          }),
        }),
      ],
    });
  }
}
