import { computed } from "mobx";
import { Float32BufferAttribute, InstancedBufferAttribute, Uint16BufferAttribute } from "three";

import { Vector2 } from "../../../../../shared/math/vector2";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { circleBufferGeometry } from "../../../three/builders";
import { instancedBufferGeometry } from "../../../three/builders";
import { instancedMesh } from "../../../three/builders";
import { mesh } from "../../../three/builders";
import { rawShader } from "../../../three/builders";
import { shaderMaterial } from "../../../three/builders";
import { withAttributes } from "../../../three/builders";
import { Canvas } from "../../../three/three";
import { CameraParams } from "../../camera_params";

import joinFragmentShader from "./shaders/join.frag";
import joinPixelVertexShader from "./shaders/join_pixel.vert";
import joinRayVertexShader from "./shaders/join_ray.vert";
import segmentFragmentShader from "./shaders/segment.frag";
import segmentPixelVertexShader from "./shaders/segment_pixel.vert";
import segmentRayVertexShader from "./shaders/segment_ray.vert";

/**
 * A line join contains a color and can be positioned either by
 * a ray from the camera's perspective, or a pixel in the image.
 */
export type LineJoinOpts = RayLineJoin | PixelLineJoin;

export interface RayLineJoin {
  ray: Vector3;
  color: Vector4;
  size?: number;
}

export interface PixelLineJoin {
  pixel: Vector2;
  color: Vector4;
  size?: number;
}

/**
 * A line has an option for looping back to the start, and is made
 * up of a list of points as either rays or pixels.
 */
export type Line = RayLine | PixelLine;

export interface RayLine {
  type: "Ray";
  joins: RayLineJoin[];
  loop?: boolean;
  width?: number;
}

export interface PixelLine {
  type: "Pixel";
  joins: PixelLineJoin[];
  loop?: boolean;
  width?: number;
}

/**
 * Options to create a group of lines, each with the same looping mode.
 * This is more efficient than drawing each line separately.
 */
export type LineGroupOpts = RayLineGroup | PixelLineGroup;

export interface RayLineGroup {
  type: "Ray";
  lines: RayLine[];
}

export interface PixelLineGroup {
  type: "Pixel";
  lines: PixelLine[];
}

/**
 * Renderer for drawing straight lines between key points.
 * Key points can be set either by rays from the camera or pixels in the camera image.
 */
export class StraightLines {
  /** Triangle indices for both a line segment and line join */
  private static readonly segmentIndices = [0, 1, 2, 0, 2, 3];

  /** Geometry for a single line segment. */
  private static readonly segmentGeometry = [
    [0, -0.5, 0],
    [1, -0.5, 0],
    [1, 0.5, 0],
    [0, 0.5, 0],
  ];

  /** UV coordinates for both a line segment and line join */
  private static readonly UVCoords = [
    [0, 0],
    [1, 0],
    [1, 1],
    [0, 1],
  ];

  constructor(
    private readonly canvas: Canvas,
    private readonly params: CameraParams,
    private readonly imageSize: Canvas,
  ) {}

  static of(opts: { canvas: Canvas; params: CameraParams; imageSize: Canvas }) {
    return new StraightLines(opts.canvas, opts.params, opts.imageSize);
  }

  @computed
  private get imageAspectRatio() {
    return this.imageSize.width / this.imageSize.height;
  }

  /**
   * Create mesh data for a group of lines.
   *
   * The positions of the lines can be set by either rays from the camera's perspective, or by pixels of the
   * image in the camera view.
   *
   * Lines in the group can each have a separate width, and can optionally loop back to their start point.
   *
   * Each join of each line can be given a size and a color. The colors of each line join is used to set the
   * color of the line segment between them.
   */
  readonly lineGroup = (opts: LineGroupOpts) => {
    if (opts.lines.length === 0) {
      return [];
    }

    switch (opts.type) {
      case "Pixel":
        return [
          this.linesMesh(opts),
          this.joinsMesh({ type: "Pixel", joins: opts.lines.flatMap((line) => line.joins) }),
        ];
      case "Ray":
        return [this.linesMesh(opts), this.joinsMesh({ type: "Ray", joins: opts.lines.flatMap((line) => line.joins) })];
    }
  };

  /** Mesh for the line segments of a group of lines */
  readonly linesMesh = mesh((opts: LineGroupOpts) => ({
    geometry: withAttributes(StraightLines.lineGeometry(), this.lineAttributes(opts)),
    material: this.material(this.segmentShader(opts)),
    frustumCulled: false,
  }));

  /** Mesh for the line joins of a group of lines */
  readonly joinsMesh = instancedMesh((opts: Line) => ({
    geometry: withAttributes(StraightLines.joinsGeometry(), this.joinsAttributes(opts)),
    material: this.material(this.joinShader(opts)),
    count: opts.joins.length,
    frustumCulled: false,
  }));

  /** Geometry for a single line segment */
  private static readonly lineGeometry = instancedBufferGeometry(() => ({
    index: new Uint16BufferAttribute(StraightLines.segmentIndices, 1),
    attributes: [
      { name: "position", buffer: new Float32BufferAttribute(StraightLines.segmentGeometry.flat(), 3) },
      { name: "uv", buffer: new Float32BufferAttribute(StraightLines.UVCoords.flat(), 2) },
    ],
  }));

  /** Geometry for a single line join */
  private static readonly joinsGeometry = circleBufferGeometry(() => ({
    radius: 0.5,
    segments: 12,
  }));

  /**
   * Attributes to add to the line segment geometry.
   * These contains the position, color, and line width data for rendering a group of lines.
   */
  private readonly lineAttributes = (opts: LineGroupOpts) => {
    const bufferData = this.lineBufferData(opts);
    return [
      { name: "startPoint", buffer: new InstancedBufferAttribute(new Float32Array(bufferData.start), 3) },
      { name: "endPoint", buffer: new InstancedBufferAttribute(new Float32Array(bufferData.end), 3) },
      { name: "startColor", buffer: new InstancedBufferAttribute(new Float32Array(bufferData.colorStart), 4) },
      { name: "endColor", buffer: new InstancedBufferAttribute(new Float32Array(bufferData.colorEnd), 4) },
      { name: "width", buffer: new InstancedBufferAttribute(new Float32Array(bufferData.width), 1) },
    ];
  };

  /**
   * Attributes to add to the line joins geometry.
   * These contains the position, color, radius of each line join in a group.
   */
  private readonly joinsAttributes = (opts: Line) => {
    const bufferData = this.lineJoinBufferData(opts);
    return [
      { name: "lineJoin", buffer: new InstancedBufferAttribute(bufferData.positions, 3) },
      { name: "color", buffer: new InstancedBufferAttribute(bufferData.colors, 4) },
      { name: "radius", buffer: new InstancedBufferAttribute(bufferData.radii, 1) },
    ];
  };

  /**
   * Create the ray and color buffer data for a line loop
   */
  private lineBufferData(opts: LineGroupOpts) {
    const bufferData = {
      colorStart: new Float32Array(opts.lines.flatMap((line) => this.lineColors(line).slice(0, -4))),
      colorEnd: new Float32Array(opts.lines.flatMap((line) => this.lineColors(line).slice(4))),
      width: new Float32Array(opts.lines.flatMap((line) => this.lineWidths(line))),
    };

    switch (opts.type) {
      case "Pixel":
        return {
          ...bufferData,
          start: new Float32Array(opts.lines.flatMap((line) => this.pixelLinePositions(line).slice(0, -3))),
          end: new Float32Array(opts.lines.flatMap((line) => this.pixelLinePositions(line).slice(3))),
        };
      case "Ray":
        return {
          ...bufferData,
          start: new Float32Array(opts.lines.flatMap((line) => this.rayLinePositions(line).slice(0, -3))),
          end: new Float32Array(opts.lines.flatMap((line) => this.rayLinePositions(line).slice(3))),
        };
      default:
        throw Error("Unknown Line Position Type");
    }
  }

  private pixelLinePositions(line: PixelLine) {
    return line.joins
      .flatMap((join) => Vector3.from(join.pixel).toArray())
      .concat(line.loop ? Vector3.from(line.joins[0].pixel).toArray() : []);
  }

  private rayLinePositions(line: RayLine) {
    return line.joins.flatMap((join) => join.ray.toArray()).concat(line.loop ? line.joins[0].ray.toArray() : []);
  }

  private lineColors(line: Line) {
    return line.joins.flatMap((join) => join.color.toArray()).concat(line.loop ? line.joins[0].color.toArray() : []);
  }

  private lineWidths(line: Line) {
    return new Array(line.loop ? line.joins.length : line.joins.length - 1).fill(line.width ?? 5);
  }

  /** Create the ray and color buffer data for the line joins of a draw */
  private lineJoinBufferData(opts: Line): { positions: Float32Array; colors: Float32Array; radii: Float32Array } {
    const colors = new Float32Array(opts.joins.flatMap((join) => join.color.toThree().toArray()));
    const radii = new Float32Array(opts.joins.flatMap((join) => join.size ?? 10));
    switch (opts.type) {
      case "Pixel":
        return { positions: new Float32Array(opts.joins.flatMap((v) => [v.pixel.x, v.pixel.y, 0])), colors, radii };
      case "Ray":
        return { positions: new Float32Array(opts.joins.flatMap((v) => [v.ray.x, v.ray.y, v.ray.z])), colors, radii };
    }
  }

  private readonly material = shaderMaterial((shader: () => THREE.RawShaderMaterial) => ({
    shader,
    uniforms: {
      Hcw: { value: this.params.Hcw.toThree() },
      viewSize: { value: new Vector2(this.canvas.width, this.canvas.height).toThree() },
      imageSize: { value: new Vector2(this.imageSize.width, this.imageSize.height).toThree() },
      imageAspectRatio: { value: this.imageAspectRatio },
      focalLength: { value: this.params.lens.focalLength },
      centre: { value: this.params.lens.centre.toThree() },
      k: { value: this.params.lens.distortionCoeffecients.toThree() },
      projection: { value: this.params.lens.projection },
    },
    depthTest: false,
    depthWrite: false,
    transparent: true,
  }));

  private segmentShader = (opts: LineGroupOpts) => {
    switch (opts.type) {
      case "Ray":
        return this.segmentRayShader;
      case "Pixel":
        return this.segmentPixelShader;
    }
  };

  private segmentRayShader = rawShader(() => ({
    vertexShader: segmentRayVertexShader,
    fragmentShader: segmentFragmentShader,
  }));

  private segmentPixelShader = rawShader(() => ({
    vertexShader: segmentPixelVertexShader,
    fragmentShader: segmentFragmentShader,
  }));

  private joinShader = (opts: Line) => {
    switch (opts.type) {
      case "Ray":
        return this.joinRayShader;
      case "Pixel":
        return this.joinPixelShader;
    }
  };

  private joinRayShader = rawShader(() => ({
    vertexShader: joinRayVertexShader,
    fragmentShader: joinFragmentShader,
  }));

  private joinPixelShader = rawShader(() => ({
    vertexShader: joinPixelVertexShader,
    fragmentShader: joinFragmentShader,
  }));
}
