import * as THREE from "three";

import { Vector3 } from "../../../../shared/math/vector3";
import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { LineProjection } from "../../camera/objects/line_projection";
import { TextViewModel } from "../../camera/objects/text/text";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";
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
  private readonly textRenderer: TextViewModel;

  constructor(
    model: BoundingBoxesModel[],
    params: CameraParams,
    lineProjection: LineProjection,
    textRenderer: TextViewModel,
  ) {
    this.model = model;
    this.params = params;
    this.lineProjection = lineProjection;
    this.textRenderer = textRenderer;
  }

  static of(
    model: BoundingBoxesModel[],
    params: CameraParams,
    canvas: Canvas,
    imageAspectRatio: number,
    imageSize: Canvas,
  ): BoundingBoxesViewModel {
    return new BoundingBoxesViewModel(
      model,
      params,
      LineProjection.of(canvas, params.lens, imageAspectRatio),
      TextViewModel.of(canvas, params, imageSize),
    );
  }

  readonly boundingBoxes = group(() => {
    const segments = new Array<THREE.Mesh>();
    const textGeometry = new Array<THREE.Object3D>(); // Array for text geometry
    this.model.forEach((m) => {
      // Loop to create lines for bounding box corners
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

      // Create and add text for the name and confidence
      const textMesh = this.textRenderer.text({
        type: "ray",
        ray: m.corners[0], // Ray from camera space
        text: `${m.name}: ${m.confidence.toFixed(2)}`,
        height: 20,
        textColor: Vector4.of(0, 0, 0, 1),
        backgroundColor: m.colour,
      });
      textGeometry.push(textMesh);
    });
    return {
      children: [...segments, ...textGeometry],
    };
  });
}
