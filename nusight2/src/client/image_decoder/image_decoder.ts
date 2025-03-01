import { IAtom } from "mobx";
import { createAtom } from "mobx";
import { createTransformer } from "mobx-utils";
import { BufferGeometry } from "three";
import { PlaneGeometry } from "three";
import { RawShaderMaterial } from "three";
import { WebGLRenderTarget } from "three";
import { Scene } from "three";
import { Mesh } from "three";
import { PixelFormat } from "three";
import { WebGLRenderer } from "three";
import { OrthographicCamera } from "three";
import { DataTexture } from "three";
import { LuminanceFormat } from "three";
import { RGBFormat } from "three";
import { Texture } from "three";
import { UnsignedByteType } from "three";
import { ClampToEdgeWrapping } from "three";
import { LinearFilter } from "three";
import { NearestFilter } from "three";
import { Camera } from "three";

import { fourccToString } from "../../shared/image_decoder/fourcc";
import { fourcc } from "../../shared/image_decoder/fourcc";

import bayerFragmentShader from "./shaders/bayer.frag";
import bayerVertexShader from "./shaders/bayer.vert";

export interface Image {
  readonly width: number;
  readonly height: number;
  readonly format: number;
  readonly data: Uint8Array;
}

export class ImageDecoder {
  private atom: IAtom;

  // These two functions are used to cache/destroy textures as they are created
  private cache: {
    get(): Texture;
    destroy(): void;
  };

  // Variables needed by the bayer decoder
  private bayerDecoder?: {
    scene: Scene;
    camera: Camera;
    geometry: BufferGeometry;
    shader: RawShaderMaterial;
  };

  constructor(private renderer: WebGLRenderer) {
    this.atom = createAtom("ImageDecoder", () => {}, this.destroy);

    this.cache = {
      get: () => new Texture(),
      destroy: () => {},
    };
  }

  static of = createTransformer(
    (renderer: WebGLRenderer) => {
      return new ImageDecoder(renderer);
    },
    (decoder) => decoder && decoder.destroy(),
  );

  private destroy = () => {
    this.cache.destroy();

    // Cleanup the bayer shader if we used it
    if (this.bayerDecoder) {
      this.bayerDecoder.geometry.dispose();
      this.bayerDecoder.shader.dispose();
    }
  };

  private updateCache(get: () => Texture, destroy: () => void) {
    // Destroy the previous texture
    this.cache.destroy();

    // Update the cache
    this.cache = { get, destroy };
  }

  // Update our stored texture
  update(image: Image): void {
    switch (image.format) {
      case fourcc("JPEG"):
        return this.jpeg(image);
      case fourcc("JPBG"):
        return this.permuted_bayer(image, [1, 1], this.mosaic_size("JPBG"));
      case fourcc("JPRG"):
        return this.permuted_bayer(image, [0, 0], this.mosaic_size("JPRG"));
      case fourcc("JPGR"):
        return this.permuted_bayer(image, [1, 0], this.mosaic_size("JPGR"));
      case fourcc("JPGB"):
        return this.permuted_bayer(image, [0, 1], this.mosaic_size("JPGB"));
      case fourcc("GRBG"):
        return this.bayer(image, [1, 0]);
      case fourcc("RGGB"):
        return this.bayer(image, [0, 0]);
      case fourcc("GBRG"):
        return this.bayer(image, [0, 1]);
      case fourcc("BGGR"):
        return this.bayer(image, [1, 1]);
      case fourcc("RGB3"):
        return this.dataTexture(image, RGBFormat);
      case fourcc("GREY"):
      case fourcc("GRAY"):
      case fourcc("Y8  "):
        return this.dataTexture(image, LuminanceFormat);
      case fourcc("PJBG"):
        return this.permuted_bayer(image, [1, 1], this.mosaic_size("PJBG"));
      case fourcc("PJRG"):
        return this.permuted_bayer(image, [0, 0], this.mosaic_size("PJRG"));
      case fourcc("PJGR"):
        return this.permuted_bayer(image, [1, 0], this.mosaic_size("PJGR"));
      case fourcc("PJGB"):
        return this.permuted_bayer(image, [0, 1], this.mosaic_size("PJGB"));
      case fourcc("PJPG"):
      case fourcc("PY8 "):
      default:
        throw Error(`Unsupported image format ${fourccToString(image.format)}`);
    }
  }

  get texture(): Texture {
    if (this.atom.reportObserved()) {
      return this.cache.get();
    } else {
      throw new Error("Atom accessed from a non reaction context");
    }
  }

  private mosaic_size(format: string): number {
    switch (fourcc(format)) {
      case fourcc("BGGR"): // Colour Bayer
      case fourcc("RGGB"): // Colour Bayer
      case fourcc("GRBG"): // Colour Bayer
      case fourcc("GBRG"): // Colour Bayer
        return 1; // One value per width/height

      case fourcc("JPBG"): // Permuted Colour Bayer
      case fourcc("JPRG"): // Permuted Colour Bayer
      case fourcc("JPGR"): // Permuted Colour Bayer
      case fourcc("JPGB"): // Permuted Colour Bayer
      case fourcc("PJPG"): // Permuted Polarized Monochrome
      case fourcc("PY8 "): // Polarized Monochrome
        return 2; // Two values per width/height

      case fourcc("PBG8"): // Polarized Colour Bayer
      case fourcc("PRG8"): // Polarized Colour Bayer
      case fourcc("PGR8"): // Polarized Colour Bayer
      case fourcc("PGB8"): // Polarized Colour Bayer
      case fourcc("PJBG"): // Permuted Polarized Colour
      case fourcc("PJRG"): // Permuted Polarized Colour
      case fourcc("PJGR"): // Permuted Polarized Colour
      case fourcc("PJGB"): // Permuted Polarized Colour
        return 4; // Four values per width/height

      default:
        return 0;
    }
  }

  private bayer(image: Image, firstRed: [number, number]): void {
    // If we have never decoded bayer with this decoder before we need to make our threejs objects
    if (!this.bayerDecoder) {
      this.bayerDecoder = {
        scene: new Scene(),
        camera: new OrthographicCamera(-1, 1, 1, -1, 0, 1),
        geometry: new PlaneGeometry(2, 2),
        shader: new RawShaderMaterial({
          vertexShader: bayerVertexShader,
          fragmentShader: bayerFragmentShader,
          depthTest: false,
          depthWrite: false,
        }),
      };
    }

    const { width, height } = image;
    const target = new WebGLRenderTarget(width, height);
    target.depthBuffer = false;
    target.stencilBuffer = false;

    // The raw bayer texture
    const rawTexture = new DataTexture(
      image.data,
      image.width,
      image.height,
      LuminanceFormat,
      UnsignedByteType,
      Texture.DEFAULT_MAPPING,
      ClampToEdgeWrapping,
      ClampToEdgeWrapping,
      NearestFilter,
      NearestFilter,
    );
    rawTexture.needsUpdate = true;

    // Set our cache to use the getter function
    this.updateCache(this.bayer_renderer(target, width, height, rawTexture, firstRed, 1), () => target.dispose());

    // The image is ready to be read now
    this.atom.reportChanged();
  }

  private permuted_bayer(image: Image, firstRed: [number, number], mosaicSize: number): void {
    // If we have never decoded bayer with this decoder before we need to make our threejs objects
    if (!this.bayerDecoder) {
      this.bayerDecoder = {
        scene: new Scene(),
        camera: new OrthographicCamera(-1, 1, 1, -1, 0, 1),
        geometry: new PlaneGeometry(2, 2),
        shader: new RawShaderMaterial({
          vertexShader: bayerVertexShader,
          fragmentShader: bayerFragmentShader,
          depthTest: false,
          depthWrite: false,
        }),
      };
    }

    const { width, height } = image;
    const target = new WebGLRenderTarget(width, height);
    target.depthBuffer = false;
    target.stencilBuffer = false;

    // Create a blob URL for our loaded data
    const blob = new Blob([image.data], { type: "image/jpeg" });
    const url = window.URL.createObjectURL(blob);

    // Create our image tag and load the url in
    const tag = document.createElement("img");

    // Don't bother doing anything until the image loads
    tag.onload = () => {
      const texture = new Texture(tag);
      texture.needsUpdate = true;
      texture.minFilter = LinearFilter;
      texture.flipY = false;

      this.updateCache(this.bayer_renderer(target, width, height, texture, firstRed, mosaicSize), () => {
        target.dispose();
        window.URL.revokeObjectURL(url);
      });

      // The image is ready to be read now
      this.atom.reportChanged();
    };
    tag.src = url;
  }

  private jpeg(image: Image): void {
    // Create a blob URL for our loaded data
    const blob = new Blob([image.data], { type: "image/jpeg" });
    const url = window.URL.createObjectURL(blob);

    // Create our image tag and load the url in
    const tag = document.createElement("img");

    // Don't bother doing anything until the image loads
    tag.onload = () => {
      const texture = new Texture(tag);
      texture.needsUpdate = true;
      texture.minFilter = LinearFilter;
      texture.flipY = false;

      this.updateCache(
        () => texture,
        () => {
          texture.dispose();
          window.URL.revokeObjectURL(url);
        },
      );

      this.atom.reportChanged();
    };
    tag.src = url;
  }

  private bayer_renderer(
    target: WebGLRenderTarget,
    width: number,
    height: number,
    texture: Texture,
    firstRed: [number, number],
    mosaicSize: number,
  ): any {
    // Create our getter function to debayer the first time it's called
    const get = () => {
      // Cloning a material allows for new uniforms without recompiling the shader itself
      const material = this.bayerDecoder!.shader.clone();
      material.uniforms.sourceSize = {
        value: [width, height, 1 / width, 1 / height],
      };
      material.uniforms.firstRed = { value: firstRed };
      material.uniforms.image = { value: texture };
      material.uniforms.mosaicSize = { value: mosaicSize };

      const mesh = new Mesh(this.bayerDecoder!.geometry, material);
      mesh.frustumCulled = false;

      // Cleanup last render
      const scene = this.bayerDecoder!.scene;
      scene.remove(...this.bayerDecoder!.scene.children);
      scene.add(mesh);

      this.renderer.setRenderTarget(target);
      this.renderer.render(scene, this.bayerDecoder!.camera);
      this.renderer.setRenderTarget(null);

      // We don't need the raw texture anymore since we already rendered the bayer
      texture.dispose();

      // Now that we have debayered once, we can just keep returning the result
      this.cache.get = () => target.texture;

      return target.texture;
    };

    return get;
  }
  private dataTexture(image: Image, format: PixelFormat): void {
    this.updateCache(
      () => {
        const texture = new DataTexture(
          image.data,
          image.width,
          image.height,
          format,
          UnsignedByteType,
          Texture.DEFAULT_MAPPING,
          ClampToEdgeWrapping,
          ClampToEdgeWrapping,
          LinearFilter,
          LinearFilter,
        );
        texture.flipY = false;
        texture.needsUpdate = true;

        // Update our cache so we don't upload to the GPU each time, and we dispose of our texture
        // Don't use update cache though as we don't want to dispose of ourself
        this.cache.get = () => texture;
        this.cache.destroy = () => {
          texture.dispose();
        };

        return texture;
      },
      () => {},
    );

    // This is ready immediately
    this.atom.reportChanged();
  }
}
