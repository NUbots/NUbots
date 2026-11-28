import * as THREE from "three";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";
import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { LineProjection } from "../../camera/objects/line_projection";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";

export type OutOfFieldFeatureStatus = "on-carpet" | "unmatched" | "candidate" | "outlier" | "mirror" | "associated";

export type OutOfFieldLandmarkStatus =
  | "not-in-view"
  | "edge"
  | "ambiguous"
  | "missed"
  | "associated"
  | "culled-missing"
  | "culled-outlier";

export interface OutOfFieldFeatureModel {
  /** Unit ray in camera space {c} towards the detected corner */
  readonly uPCc: Vector3;
  readonly status: OutOfFieldFeatureStatus;
}

export interface OutOfFieldLandmarkModel {
  /** Unit ray in {c} towards where the landmark is predicted to appear */
  readonly uPCc: Vector3;
  readonly status: OutOfFieldLandmarkStatus;
  /** Unit ray in {c} towards the corner that claimed it, when associated */
  readonly uMatchCc?: Vector3;
  readonly bearingOnly: boolean;
}

export interface OutOfFieldModel {
  readonly timestamp: number;
  readonly Hcw: Matrix4;
  readonly features: OutOfFieldFeatureModel[];
  readonly landmarks: OutOfFieldLandmarkModel[];
  /** Accumulated own-vs-mirror log-likelihood ratio [nats]; positive backs the current pose */
  readonly llr: number;
  readonly mapFrozen: boolean;
  readonly flipRequested: boolean;
}

/**
 * Colour key. Cyan: unexplained background corner. Steel: candidate track. Green: matched at the
 * current pose. Yellow: matched only at the mirrored pose (wrong-side evidence). Red: rejected.
 * Grey (faint): masked as carpet.
 */
const featureColour: Record<OutOfFieldFeatureStatus, Vector4> = {
  "on-carpet": new Vector4(0.35, 0.35, 0.35, 0.6),
  unmatched: new Vector4(0.0, 1.0, 1.0, 1.0),
  candidate: new Vector4(0.31, 0.67, 0.9, 1.0),
  outlier: new Vector4(1.0, 0.24, 0.24, 1.0),
  mirror: new Vector4(0.92, 0.92, 0.0, 1.0),
  associated: new Vector4(0.47, 0.9, 0.47, 1.0),
};

const landmarkColour: Record<OutOfFieldLandmarkStatus, Vector4> = {
  "not-in-view": new Vector4(0.29, 0.31, 0.33, 1.0),
  edge: new Vector4(0.51, 0.51, 0.51, 1.0),
  ambiguous: new Vector4(0.78, 0.55, 0.78, 1.0),
  missed: new Vector4(1.0, 0.75, 0.24, 1.0),
  associated: new Vector4(0.47, 0.9, 0.47, 1.0),
  "culled-missing": new Vector4(0.51, 0.51, 0.51, 1.0),
  "culled-outlier": new Vector4(1.0, 0.24, 0.24, 1.0),
};

/**
 * Angular radius of each marker [rad]. Drawn as cones about camera rays so markers follow the lens
 * distortion.
 */
const featureRadius: Record<OutOfFieldFeatureStatus, number> = {
  "on-carpet": 0.0015,
  unmatched: 0.004,
  candidate: 0.004,
  outlier: 0.006,
  mirror: 0.006,
  associated: 0.004,
};

/** Wider than the corner markers, so a prediction reads as a ring the matched corner sits inside. */
const landmarkRadius = 0.011;

export class OutOfFieldViewModel {
  constructor(
    private readonly model: OutOfFieldModel,
    private readonly params: CameraParams,
    private readonly lineProjection: LineProjection,
  ) {}

  static of(
    model: OutOfFieldModel,
    params: CameraParams,
    canvas: Canvas,
    imageAspectRatio: number,
  ): OutOfFieldViewModel {
    return new OutOfFieldViewModel(model, params, LineProjection.of(canvas, params.lens, imageAspectRatio));
  }

  readonly outOfField = group(() => {
    // Landmarks first so corner markers draw on top
    const Hcc = this.Hcc;
    return { children: [...this.landmarkObjects(Hcc), ...this.featureObjects(Hcc)] };
  });

  /**
   * Rays are relative to the measurement's Hcw, which may lag the displayed image; re-express them
   * in the current camera.
   */
  private get Hcc(): THREE.Matrix4 {
    const Hwc = new THREE.Matrix4().copy(this.model.Hcw.toThree()).invert();
    return this.params.Hcw.toThree().clone().multiply(Hwc);
  }

  private transform(ray: Vector3, Hcc: THREE.Matrix4): Vector3 {
    return Vector3.fromThree(ray.toThree().applyMatrix4(Hcc).normalize());
  }

  private featureObjects(Hcc: THREE.Matrix4): THREE.Object3D[] {
    return this.model.features.map((feature) =>
      this.lineProjection.cone({
        axis: this.transform(feature.uPCc, Hcc),
        radius: Math.cos(featureRadius[feature.status]),
        color: featureColour[feature.status],
        lineWidth: feature.status === "on-carpet" ? 2 : 4,
      }),
    );
  }

  private landmarkObjects(Hcc: THREE.Matrix4): THREE.Object3D[] {
    const objects: THREE.Object3D[] = [];
    for (const landmark of this.model.landmarks) {
      if (landmark.status === "not-in-view") {
        continue;
      }
      const axis = this.transform(landmark.uPCc, Hcc);
      const colour = landmarkColour[landmark.status];
      objects.push(
        this.lineProjection.cone({
          axis,
          radius: Math.cos(landmarkRadius),
          color: colour,
          // Bearing-only landmarks are drawn thinner, as their position is far less certain
          lineWidth: landmark.bearingOnly ? 2 : 4,
        }),
      );
      // Residual from the prediction to the matched corner
      if (landmark.uMatchCc) {
        objects.push(
          this.lineProjection.planeSegment({
            start: axis,
            end: this.transform(landmark.uMatchCc, Hcc),
            color: colour,
            lineWidth: 2,
          }),
        );
      }
    }
    return objects;
  }
}
