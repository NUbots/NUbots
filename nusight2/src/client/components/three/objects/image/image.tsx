import React, { useMemo } from "react";
import { BayerImageFormat, Image, ImageFormat } from "@components/camera/image";
import { ShaderUniform } from "@components/three/shader_uniform";
import { useUpdatable } from "@hooks/use_updatable";
import { UnreachableError } from "@shared/base/unreachable_error";
import * as THREE from "three";

import fragmentShader from "./shaders/bayer.frag";
import vertexShader from "./shaders/bayer.vert";
import { getFirstRed, getImageData, getMosaicSize, getTextureFormat } from "./utils";

/**
 * Renders an image.
 *
 * The center of the geometry is at (0, 0) and the width and height are those of the image's.
 *
 * Children of this component are transformed to be in the image's pixel coordinate
 * space so that (0, 0) is at the top-left of the image and (image.width, image.height)
 * is at the bottom right of the image.
 */
export function ImageView(props: { image: Image; visible?: boolean; children?: React.ReactNode }) {
  const { image, visible = true, children } = props;
  return (
    // Transform to center the image and set the positive direction down
    <group position={[-image.width / 2, image.height / 2, 0]} scale={[1, -1, 1]}>
      <mesh visible={visible} frustumCulled={false}>
        <ImageGeometry width={image.width} height={image.height} />
        <ImageMaterial image={image} />
      </mesh>
      {children}
    </group>
  );
}

function ImageGeometry(props: { width: number; height: number }) {
  const { width, height } = props;

  // Build geometry such that the width and height equal the image dimensions and (0, 0) is the
  // top-left corner of the geometry
  const geometry = useMemo(() => {
    // prettier-ignore
    return {
      vertices: [[0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0]].flat(),
      uv: [[0, 0], [1, 0], [1, 1], [0, 1]].flat(),
      indices: [0, 1, 2, 0, 2, 3],
    };
  }, [width, height]);

  return (
    <bufferGeometry>
      <float32BufferAttribute attach="attributes-position" args={[geometry.vertices, 3]} />
      <float32BufferAttribute attach="attributes-uv" args={[geometry.uv, 2]} />
      <uint16BufferAttribute attach="index" args={[geometry.indices, 1]} />
    </bufferGeometry>
  );
}

/** Get a texture that updates its properties each time the given image changes */
export function useImageTexture(image: Image, opts?: { filtering?: THREE.MagnificationTextureFilter }) {
  const create = ([image, filtering]: [Image, THREE.MagnificationTextureFilter | undefined]): THREE.Texture => {
    const texture = new THREE.Texture();
    texture.image = getImageData(image);
    texture.format = getTextureFormat(image.format);
    texture.generateMipmaps = false;
    texture.format = THREE.RGBAFormat;
    texture.type = THREE.UnsignedByteType;
    texture.magFilter = filtering ?? THREE.NearestFilter;
    texture.minFilter = filtering ?? THREE.NearestFilter;
    texture.needsUpdate = true;
    texture.flipY = false;
    return texture;
  };

  const update = (
    texture: THREE.Texture,
    [image, filtering]: [Image, THREE.MagnificationTextureFilter | undefined],
  ) => {
    // Close the previous bitmap to avoid GPU memory leaks
    if (texture.image instanceof ImageBitmap) {
      texture.image.close();
    }
    texture.image = getImageData(image);
    texture.format = getTextureFormat(image.format);
    texture.generateMipmaps = false;
    texture.format = THREE.RGBAFormat;
    texture.type = THREE.UnsignedByteType;
    texture.magFilter = filtering ?? THREE.NearestFilter;
    texture.minFilter = filtering ?? THREE.NearestFilter;
    texture.needsUpdate = true;
    texture.flipY = false;
  };

  return useUpdatable(create, update, [image, opts?.filtering] as [
    Image,
    THREE.MagnificationTextureFilter | undefined,
  ]);
}

/** Material for images with a non-bayer pixel format */
function BasicMaterial(props: { image: Image }) {
  const { image } = props;
  const texture = useImageTexture(image);

  return <meshBasicMaterial map={texture} />;
}

/** Material for images with a bayer pixel format */
function BayerMaterial(props: { image: Image; format: BayerImageFormat }) {
  const { image, format } = props;
  const texture = useImageTexture(image);

  const { firstRed, mosaicSize } = useMemo(
    () => ({ firstRed: getFirstRed(format), mosaicSize: getMosaicSize(format) }),
    [format],
  );

  return (
    <rawShaderMaterial vertexShader={vertexShader} fragmentShader={fragmentShader} transparent>
      <ShaderUniform name="image" value={texture} />
      <ShaderUniform name="sourceSize" value={[image.width, image.height, 1 / image.width, 1 / image.height]} />
      <ShaderUniform name="firstRed" value={firstRed} />
      <ShaderUniform name="mosaicSize" value={mosaicSize} />
    </rawShaderMaterial>
  );
}

function ImageMaterial(props: { image: Image }) {
  const { image } = props;
  switch (image.format) {
    case ImageFormat.JPEG:
    case ImageFormat.RGB8:
    case ImageFormat.GREY:
    case ImageFormat.GRAY:
    case ImageFormat.Y8__:
      // Use 'key' to force texture to be rebuilt when the width or height change.
      // Three JS textures can't have their image be replaced by one with a higher
      // resolution than the initial image.
      return <BasicMaterial key={image.width + "," + image.height} image={image} />;
    case ImageFormat.JPBG:
    case ImageFormat.JPRG:
    case ImageFormat.JPGR:
    case ImageFormat.JPGB:
    case ImageFormat.GRBG:
    case ImageFormat.RGGB:
    case ImageFormat.GBRG:
    case ImageFormat.BGGR:
    case ImageFormat.PJBG:
    case ImageFormat.PJRG:
    case ImageFormat.PJGR:
    case ImageFormat.PJGB:
      // Use 'key' to force texture to be rebuilt when the width or height change.
      // Three JS textures can't have their image be replaced by one with a higher
      // resolution than the initial image.
      return <BayerMaterial key={image.width + "," + image.height} image={image} format={image.format} />;
    default:
      throw new UnreachableError(image.format);
  }
}
