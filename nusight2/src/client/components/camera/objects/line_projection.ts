import * as THREE from "three";

import { Vector2 } from "../../../../shared/math/vector2";
import { Vector3 } from "../../../../shared/math/vector3";
import { Vector4 } from "../../../../shared/math/vector4";
import { mesh } from "../../three/builders";
import { planeGeometry } from "../../three/builders";
import { rawShader } from "../../three/builders";
import { shaderMaterial } from "../../three/builders";
import { Canvas } from "../../three/three";
import { Lens } from "../lens";

import fragmentShader from "./shaders/line_projection.frag";
import vertexShader from "./shaders/line_projection.vert";

export interface ConeSegment {
  /** The camera space central axis of the cone. */
  axis: Vector3;
  /** The camera space vector pointing to the start of the segment. */
  start: Vector3;
  /** The camera space vector pointing to the end of the segment. */
  end: Vector3;
  /** The colour of the line to draw. */
  color?: Vector4;
  /** The width of the line to draw on the screen in pixels. */
  lineWidth?: number;
}

export class LineProjection {
  constructor(
    private readonly canvas: Canvas,
    private readonly lens: Lens,
    private readonly imageAspectRatio: number,
  ) {}

  static of(canvas: Canvas, lens: Lens, imageAspectRatio: number) {
    return new LineProjection(canvas, lens, imageAspectRatio);
  }

  /** Draws a plane projected to infinity in world space. */
  plane({ axis, color, lineWidth }: { axis: Vector3; color?: Vector4; lineWidth?: number }) {
    // Pick an arbitrary orthogonal vector
    const start = Vector3.fromThree(
      !axis.x && !axis.y ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(-axis.y, axis.x, 0).normalize(),
    );
    return this.coneSegment({ axis, start, end: start, color, lineWidth });
  }

  /** Draws a segment of a plane projected to infinity in world space. */
  planeSegment(segment: { axis?: Vector3; start: Vector3; end: Vector3; color?: Vector4; lineWidth?: number }) {
    return this.coneSegment({
      ...segment,
      axis:
        segment.axis ??
        Vector3.fromThree(new THREE.Vector3().crossVectors(segment.start.toThree(), segment.end.toThree()).normalize()),
    });
  }

  /** Draw a cone projected to infinity in world space. Only draws the positive cone, not the negative cone. */
  cone({ axis, radius, color, lineWidth }: { axis: Vector3; radius: number; color?: Vector4; lineWidth?: number }) {
    // Pick an arbitrary orthogonal vector
    const orth = !axis.x && !axis.y ? new Vector3(0, 1, 0) : new Vector3(-axis.y, axis.x, 0).normalize();
    // Rotate our axis by this radius to get a start
    const start = Vector3.fromThree(axis.toThree().applyAxisAngle(orth.toThree(), Math.acos(radius)));
    return this.coneSegment({ axis, start, end: start, color, lineWidth });
  }

  readonly coneSegment = mesh((segment: ConeSegment) => ({
    geometry: LineProjection.geometry(),
    material: this.material(segment),
  }));

  private readonly material = shaderMaterial((segment: ConeSegment) => {
    const { projection, focalLength, centre = Vector2.of(), distortionCoeffecients = Vector2.of() } = this.lens;
    return {
      shader: LineProjection.shader,
      uniforms: {
        viewSize: { value: new THREE.Vector2(this.canvas.width, this.canvas.height) },
        projection: { value: projection },
        focalLength: { value: focalLength },
        centre: { value: centre.toThree() },
        k: { value: distortionCoeffecients.toThree() },
        imageAspectRatio: { value: this.imageAspectRatio },
        axis: { value: segment.axis?.toThree() },
        start: { value: segment.start.toThree() },
        end: { value: segment.end.toThree() },
        color: { value: segment.color?.toThree() },
        lineWidth: { value: segment.lineWidth },
      },
      depthTest: false,
      depthWrite: false,
      transparent: true,
    };
  });

  private static readonly shader = rawShader(() => ({ vertexShader, fragmentShader }));

  private static readonly geometry = planeGeometry(() => ({
    width: 2,
    height: 2,
    widthSegments: 100,
  }));
}
