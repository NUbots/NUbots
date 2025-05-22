import { computed } from "mobx";
import * as THREE from "three";

import { Vector2 } from "../../../../../shared/math/vector2";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { bufferGeometry, imageTexture, mesh, rawShader, shaderMaterial } from "../../../three/builders";
import { Canvas } from "../../../three/three";
import { CameraParams } from "../../camera_params";

import fragmentShader from "./shaders/text.frag";
import vertexShader from "./shaders/text.vert";

/**
 * Offset of a character in the text texture
 * and its normalised width in the texture.
 */
interface CharUVOffset {
  offset: number;
  width: number;
}

type RayPosition = {
  type: "ray";

  /**
   * Ray from the camera's perspective used to position the text.
   * Defaults to forward direction.
   */
  ray: Vector3;

  /** Offset in pixels for where to position the text. */
  pixelOffset?: Vector2;
};

type ImagePosition = {
  type: "pixel";

  /** Pixel of the image to position the text. */
  pixel: Vector2;
};

export type TextOpts = (RayPosition | ImagePosition) & {
  /** Text to render */
  text: string;

  /** Colour of the text. Defaults to white. */
  textColor?: Vector4;

  /** Colour of the background. Defaults to transparent */
  backgroundColor?: Vector4;

  /** Height of the text geometry in display pixels. */
  height?: number;

  /**
   * Horizontal alignment of the text. This determines how the text geometry is positioned
   * relative to the image pixel or ray.
   */
  align?: "start" | "middle" | "end";

  /**
   * Vertical alignment of the text. This determines how the text geometry is positioned
   * relative to the image pixel or ray.
   */
  baseline?: "top" | "middle" | "bottom";
};

/** Default values for text generation */
const DefaultOpts = {
  textColor: Vector4.of(1, 1, 1, 1),
  backgroundColor: Vector4.of(0, 0, 0, 0),
  height: 20,
} as const;

export class TextViewModel {
  /** Size of font for text drawn to the textures. */
  private static readonly textSize = 30;

  /**
   * Textures are generated when the first instance of a Text Renderer is created.
   *
   * All instances share the same textures, preventing them from being generated
   * each time an instance is created.
   */
  private static createdMask = false;

  /**
   * A mask texture containing the shapes of the valid range of character codes.
   *
   * During rendering, the final color is determined by interpolating from the
   * `textColor` to the `backgroundColor` using the alpha values of this texture.
   */
  private static canvasCharMask: HTMLCanvasElement;

  /**
   * Mapping between characters and their x offset in the character mask.
   *
   * This is used to create the width of each quad in the geometry and the UV map
   * for the text.
   */
  private static readonly charUVMap: Map<string, CharUVOffset> = new Map();

  constructor(
    private readonly canvas: Canvas,
    private readonly params: CameraParams,
    private readonly imageSize: Canvas,
  ) {
    // On the first created instance, create the texture for the font
    if (!TextViewModel.createdMask) {
      TextViewModel.createdMask = true;

      // String containing all the characters of utf-16 codes in useful range
      const charCodeStart = 32; //' '
      const charCodeEnd = 127; //'~'
      const characterSet = String.fromCharCode(...Array(charCodeEnd).keys()).slice(charCodeStart) + "□";

      // Insert small space between characters to prevent them overlapping
      const spacedChars = characterSet.split("").join(" ");

      TextViewModel.createCharUVMap(spacedChars);
      TextViewModel.createCharMask(spacedChars);
    }
  }

  static of(canvas: Canvas, params: CameraParams, imageSize: Canvas) {
    return new TextViewModel(canvas, params, imageSize);
  }

  @computed
  private get imageAspectRatio() {
    return this.imageSize.width / this.imageSize.height;
  }

  /** Calculates the offset of each character in the font texture. */
  private static createCharUVMap(charString: string) {
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d")!;
    TextViewModel.setAttributes(ctx);

    const width = ctx.measureText(charString).width;
    let offset = 0;
    for (const char of charString) {
      const charWidth = ctx!.measureText(char).width / width;
      TextViewModel.charUVMap.set(char, { offset: offset, width: charWidth });
      offset += charWidth;
    }
  }

  /**
   * Create the mask for the character set.
   *
   * This mask has a transparent background with the entire set of characters
   * drawn on it with an alpha of 1.
   */
  private static createCharMask(charString: string) {
    TextViewModel.canvasCharMask = document.createElement("canvas");
    const ctx = TextViewModel.canvasCharMask.getContext("2d")!;
    this.setAttributes(ctx);

    TextViewModel.canvasCharMask.width = ctx.measureText(charString).width;
    TextViewModel.canvasCharMask.height = TextViewModel.textSize + 8;
    this.setAttributes(ctx);

    // Fully transparent background
    ctx.globalAlpha = 0;
    ctx.fillStyle = "black";
    ctx.fillRect(0, 0, TextViewModel.canvasCharMask.width, TextViewModel.canvasCharMask.height);

    // Draw characters onto the texture in white
    ctx.globalAlpha = 1;
    ctx.fillStyle = "white";
    ctx.fillText(charString, 0, TextViewModel.canvasCharMask.height - 2);
  }

  private static setAttributes(ctx: CanvasRenderingContext2D) {
    ctx.font = `${TextViewModel.textSize}px Roboto, ui-sans-serif, system-ui`;
    ctx.textBaseline = "bottom";
  }

  readonly text = (opts: TextOpts) => {
    return mesh(() => ({
      geometry: this.geometry(opts),
      material: this.material(opts),
      frustumCulled: false,
    }))();
  };

  private positionMode(opts: TextOpts) {
    switch (opts.type) {
      case "ray":
        return 0;
      case "pixel":
        return 1;
    }
  }

  private readonly textureImage = imageTexture(() => ({
    image: TextViewModel.canvasCharMask,
    format: THREE.RGBAFormat,
  }));

  private readonly material = shaderMaterial((opts: TextOpts) => {
    const ray = opts.type === "ray" ? opts.ray : Vector3.of();
    const pixel = opts.type === "pixel" ? opts.pixel : (opts.pixelOffset ?? Vector2.of());
    return {
      shader: TextViewModel.shader,
      uniforms: {
        texture: { value: this.textureImage() },
        Hcw: { value: this.params.Hcw.toThree() },
        viewSize: { value: new Vector2(this.canvas.width, this.canvas.height).toThree() },
        imageSize: { value: new Vector2(this.imageSize.width, this.imageSize.height).toThree() },
        imageAspectRatio: { value: this.imageAspectRatio },
        focalLength: { value: this.params.lens.focalLength },
        centre: { value: this.params.lens.centre.toThree() },
        k: { value: this.params.lens.distortionCoeffecients.toThree() },
        projection: { value: this.params.lens.projection },
        textColor: { value: opts.textColor?.toThree() ?? DefaultOpts.textColor.toThree() },
        backgroundColor: { value: opts.backgroundColor?.toThree() ?? DefaultOpts.backgroundColor.toThree() },
        textHeight: { value: opts.height ?? DefaultOpts.height },
        ray: { value: ray.toThree() },
        pixel: { value: pixel.toThree() },
        positionMode: { value: this.positionMode(opts) },
      },
      depthTest: false,
      depthWrite: false,
      transparent: true,
    };
  });

  /** Multiplier for the text geometry start position to apply the horizontal alignment */
  private getGeometryXMultiplier(align: TextOpts["align"]): number {
    switch (align) {
      case "end":
        return -1;
      case "middle":
        return -0.5;
      case "start":
      default:
        return 0;
    }
  }

  /** Y Position for the text renderer with the vertical alignment applied */
  private getGeometryY(baseline: TextOpts["baseline"]): number {
    switch (baseline) {
      case "top":
        return -1;
      case "middle":
        return -0.5;
      case "bottom":
      default:
        return 0;
    }
  }

  private readonly geometry = bufferGeometry((opts: TextOpts) => {
    const vertexBufferData = [];
    const uvBufferData = [];

    const aspectRatio = TextViewModel.canvasCharMask.width / TextViewModel.canvasCharMask.height;

    // Pad text sides so text doesn't touch the edges
    const text = ` ${opts.text} `;

    // Get the data for each char of the text
    const charData = Array.from(text).map((char) => TextViewModel.charUVMap.get(char));

    // Calculate the total width of the text geometry
    const charWidths = charData.map((c) => c?.width ?? 0);
    const totalWidth = charWidths.reduce((acc: number, val: number) => acc + val, 0);

    let xPos = this.getGeometryXMultiplier(opts.align) * totalWidth * aspectRatio;
    const yPos = this.getGeometryY(opts.baseline);

    for (let char of charData) {
      // If we don't have data for a char, render box instead
      if (!char) {
        char = TextViewModel.charUVMap.get("□")!;
      }

      const nextVertex = xPos + char.width * aspectRatio;

      // Quad for this character
      //prettier-ignore
      vertexBufferData.push(
        xPos, yPos, 0,
        nextVertex, yPos + 1, 0,
        xPos, yPos + 1, 0,
        xPos, yPos, 0,
        nextVertex, yPos, 0,
        nextVertex, yPos + 1, 0,
      );

      // UV mapping for this character
      //prettier-ignore
      uvBufferData.push(
        char.offset, 1,
        char.offset + char.width, 0,
        char.offset, 0,
        char.offset, 1,
        char.offset + char.width, 1,
        char.offset + char.width, 0,
      );
      xPos = nextVertex;
    }

    return {
      attributes: [
        { name: "position", buffer: new THREE.BufferAttribute(new Float32Array(vertexBufferData), 3) },
        { name: "uv", buffer: new THREE.BufferAttribute(new Float32Array(uvBufferData), 2) },
      ],
    };
  });

  private static readonly shader = rawShader(() => ({ vertexShader, fragmentShader }));
}
