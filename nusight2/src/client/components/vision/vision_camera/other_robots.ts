import { createTransformer } from "mobx-utils";
import * as THREE from "three";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Vector3 } from "../../../../shared/math/vector3";
import { Vector4 } from "../../../../shared/math/vector4";
import { CameraParams } from "../../camera/camera_params";
import { LineProjection } from "../../camera/objects/line_projection";
import { group } from "../../three/builders";
import { Canvas } from "../../three/three";

const ROBOT_COLOUR = new Vector4(1, 0.5, 0, 1); // orange

export interface OtherRobotsModel {
  readonly timestamp: number;
  readonly Hcw: Matrix4;
  readonly rRCc: Vector3;
  readonly radius: number;
}

export class OtherRobotsViewModel {
  private readonly model: OtherRobotsModel[];
  private readonly params: CameraParams;
  private readonly lineProjection: LineProjection;

  constructor(model: OtherRobotsModel[], params: CameraParams, lineProjection: LineProjection) {
    this.model = model;
    this.params = params;
    this.lineProjection = lineProjection;
  }

  static of(
    model: OtherRobotsModel[],
    params: CameraParams,
    canvas: Canvas,
    imageAspectRatio: number,
  ): OtherRobotsViewModel {
    return new OtherRobotsViewModel(model, params, LineProjection.of(canvas, params.lens, imageAspectRatio));
  }

  readonly robots = group(() => ({
    children: this.model.map((robot) => this.robot(robot)),
  }));

  private robot = createTransformer((m: OtherRobotsModel) => {
    const Hwc = new THREE.Matrix4().copy(m.Hcw.toThree()).invert();
    const Hcc = Matrix4.fromThree(this.params.Hcw.toThree().multiply(Hwc));

    // Transform the robot centre so it is in the perspective of the latest camera image.
    const rRCc = Vector3.fromThree(m.rRCc.toThree().applyMatrix4(Hcc.toThree()));
    const Rcw = new THREE.Matrix3().setFromMatrix4(this.params.Hcw.toThree());

    // We get normal to ground in world space, then view the normal to ground in camera space
    const groundNormalC = new THREE.Vector3(0, 0, 1).applyMatrix3(Rcw).normalize();

    // Build an orthonormal basis, spanning the ground plane in camera coordinates.
    const basisSeed = Math.abs(groundNormalC.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
    const u = new THREE.Vector3().crossVectors(groundNormalC, basisSeed).normalize();
    const v = new THREE.Vector3().crossVectors(groundNormalC, u).normalize();

    // Sample points on a 3D circle (robots footprint)
    const raySamples = 32;
    const angleDelta = (2 * Math.PI) / raySamples;
    const centre = rRCc.normalize().toThree();
    const rays = new Array(raySamples).fill(0).map((_, i) => {
      const t = i * angleDelta;
      const pointC = centre
        .clone()
        .add(u.clone().multiplyScalar(Math.acos(m.radius) * Math.cos(t)))
        .add(v.clone().multiplyScalar(Math.acos(m.radius) * Math.sin(t)));
      return Vector3.fromThree(pointC.normalize());
    });

    return this.lineProjection.rayLoop({
      rays: rays,
      color: ROBOT_COLOUR,
      lineWidth: 4,
    });
  });
}
