import { createTransformer } from "mobx-utils";
import * as THREE from "three";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";
import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { LineProjection } from "../../camera/objects/line_projection";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";

const BOX_COLOUR = new Vector4(1, 0.5, 0, 1); // orange

export interface BoundingBoxesModel {
  readonly timestamp: number;
  readonly name: string;
  readonly confidence: number;
  readonly corners: Vector3[];
  readonly colour: Vector4;
}

export class BoundingBoxesViewModel {
  private readonly model: BoundingBoxesModel[];
  private readonly params: CameraParams;
  private readonly lineProjection: LineProjection;

  constructor(model: BoundingBoxesModel[], params: CameraParams, lineProjection: LineProjection) {
    this.model = model;
    this.params = params;
    this.lineProjection = lineProjection;
  }

  static of(
    model: BoundingBoxesModel[],
    params: CameraParams,
    canvas: Canvas,
    imageAspectRatio: number,
  ): BoundingBoxesViewModel {
    return new BoundingBoxesViewModel(model, params, LineProjection.of(canvas, params.lens, imageAspectRatio));
  }

  readonly boundingBoxes = group(() => {
    // Create bounding box with name and confidence, loop through corners and draw lines between them
    const segments = new Array<THREE.Mesh>();
    this.model.forEach((m) => {
      for (let i = 0; i < 4; i++) {
        segments.push(
          this.lineProjection.planeSegment({
            start: m.corners[i],
            end: m.corners[(i + 1) % 4],
            color: m.colour,
            lineWidth: 3,
          }),
        );
      }
    });
    return {
      children: segments,
    };
  });
}
