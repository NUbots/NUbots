import { createTransformer } from "mobx-utils";
import * as THREE from "three";

import { UnreachableError } from "../../../../shared/base/unreachable_error";
import { FieldDimensions } from "../../../../shared/field/dimensions";
import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";
import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { LineProjection } from "../../camera/objects/line_projection";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";

export interface GoalModel {
  readonly timestamp: number;
  readonly Hcw: Matrix4;
  readonly side: "left" | "right" | "unknown";
  readonly post: {
    readonly top: Vector3;
    readonly bottom: Vector3;
    readonly distance: number;
  };
}

export class GoalsViewModel {
  private readonly model: GoalModel[];
  private readonly params: CameraParams;
  private readonly lineProjection: LineProjection;

  constructor(model: GoalModel[], params: CameraParams, lineProjection: LineProjection) {
    this.model = model;
    this.params = params;
    this.lineProjection = lineProjection;
  }

  static of(model: GoalModel[], params: CameraParams, canvas: Canvas, imageAspectRatio: number): GoalsViewModel {
    return new GoalsViewModel(model, params, LineProjection.of(canvas, params.lens, imageAspectRatio));
  }

  readonly goals = group(() => ({
    children: this.model.map((goal) => this.goal(goal)),
  }));

  private goal = createTransformer((goal: GoalModel) => {
    // Transform the goal line so that it is in the perspective of the latest camera image.
    const Hwc = new THREE.Matrix4().copy(goal.Hcw.toThree()).invert();
    const Hcc = this.params.Hcw.toThree().multiply(Hwc);
    const bottom = goal.post.bottom.toThree().multiplyScalar(goal.post.distance);
    return this.lineProjection.planeSegment({
      start: Vector3.fromThree(
        bottom
          .clone()
          .applyMatrix4(Hwc)
          .add(new THREE.Vector3(0, 0, FieldDimensions.postYear2017().goalCrossbarHeight))
          .applyMatrix4(this.params.Hcw.toThree())
          .normalize(),
      ),
      end: Vector3.fromThree(bottom.clone().applyMatrix4(Hcc).normalize()),
      color: getColor(goal.side),
      lineWidth: 10,
    });
  });
}

function getColor(side: GoalModel["side"]) {
  switch (side) {
    case "left":
      return new Vector4(1.0, 1.0, 0, 1.0); // Yellow
    case "right":
      return new Vector4(0.0, 1.0, 1.0, 1.0); // Cyan
    case "unknown":
      return new Vector4(1.0, 0.0, 1.0, 1.0); // Magenta
    default:
      throw new UnreachableError(side);
  }
}
