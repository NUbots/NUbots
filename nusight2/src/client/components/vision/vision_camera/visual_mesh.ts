import * as THREE from "three";

import { Vector2 } from "../../../../shared/math/vector2";
import { CameraParams } from "../../camera/camera_params";
import { bufferGeometry, mesh, rawShader, shaderMaterial } from "../../three/builders";
import { Canvas } from "../../three/three";

import fragmentShader from "./shaders/visual_mesh.frag";
import vertexShader from "./shaders/visual_mesh.vert";

export interface VisualMeshModel {
  readonly neighbours: number[];
  readonly rays: number[];
  readonly classifications: { dim: number; values: number[] };
}

export class VisualMeshViewModel {
  private readonly model: VisualMeshModel;
  private readonly params: CameraParams;
  private readonly canvas: Canvas;
  private readonly imageAspectRatio: number;

  constructor(model: VisualMeshModel, params: CameraParams, canvas: Canvas, imageAspectRatio: number) {
    this.model = model;
    this.params = params;
    this.canvas = canvas;
    this.imageAspectRatio = imageAspectRatio;
  }

  static of(model: VisualMeshModel, params: CameraParams, canvas: Canvas, imageAspectRatio: number) {
    return new VisualMeshViewModel(model, params, canvas, imageAspectRatio);
  }

  readonly visualMesh = mesh(() => ({
    geometry: this.geometry(),
    material: this.material(),
  }));

  private readonly geometry = bufferGeometry(() => {
    const { neighbours, rays, classifications } = this.model;

    // Calculate our triangle indexes
    const nElem = rays.length / 3;
    const triangles = [];
    for (let i = 0; i < nElem; i++) {
      const ni = i * 6;

      if(neighbours[ni + 0] < nElem && neighbours[ni + 1] < nElem) {
        triangles.push(i, neighbours[ni + 0], neighbours[ni + 1]);
        triangles.push(i, neighbours[ni + 1], neighbours[ni + 0])
      }
      if(neighbours[ni + 1] < nElem && neighbours[ni + 2] < nElem) {
        triangles.push(i, neighbours[ni + 1], neighbours[ni + 2]);
        triangles.push(i, neighbours[ni + 2], neighbours[ni + 1])
      }
    }
    const buffer = new THREE.InterleavedBuffer(
      new Float32Array(classifications.values.slice(0, -classifications.dim)),
      classifications.dim,
    );

    return {
      index: triangles,
      attributes: [
        { name: "position", buffer: new THREE.Float32BufferAttribute(rays, 3) },
        { name: "ball", buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 0) },
        { name: "goal", buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 1) },
        { name: "fieldLine", buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 2) },
        { name: "field", buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 3) },
        { name: "environment", buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 4) },
      ],
    };
  });

  private readonly material = shaderMaterial(() => ({
    shader: VisualMeshViewModel.shader,
    uniforms: {
      Hcw: { value: this.params.Hcw.toThree() },
      viewSize: { value: new Vector2(this.canvas.width, this.canvas.height).toThree() },
      focalLength: { value: this.params.lens.focalLength },
      centre: { value: this.params.lens.centre.toThree() },
      k: { value: this.params.lens.distortionCoeffecients.toThree() },
      imageAspectRatio: { value: this.imageAspectRatio },
      projection: { value: this.params.lens.projection },
    },
    depthTest: false,
    depthWrite: false,
    transparent: true,
  }));

  private static readonly shader = rawShader(() => ({ vertexShader, fragmentShader }));
}
