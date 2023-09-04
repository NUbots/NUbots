import React from "react";
import { useThree } from "@react-three/fiber";
import { observer } from "mobx-react";
import * as THREE from "three";

import { UnreachableError } from "../../../../../shared/base/unreachable_error";
import { ImageFormat } from "../image";
import { BayerImageFormat } from "../image";
import { Image } from "../image";

import fragmentShader from "./shaders/bayer.frag";
import vertexShader from "./shaders/bayer.vert";

export const ImageView = ({ image }: { image: Image }) => {
  const {
    size: { width, height },
  } = useThree();
  // Aspect ratio of canvas / image
  const canvasRatio = width !== 0 ? width / height : 1;
  const sourceRatio = image.width / image.height;

  // Depending on which aspect ratio is wider,
  // scale either the width or height of the image geometry,
  // so that it maintains the correct aspect ratio.
  const geometry =
    canvasRatio > sourceRatio
      ? { width: 2 * (sourceRatio / canvasRatio), height: 2 }
      : { width: 2, height: 2 * (canvasRatio / sourceRatio) };

  // Normally this effect could be achieved by setting texture.flipY to make
  // the textures the correct way up again. However this is ignored on RenderTargets
  // We can't flip it at the raw stage either as this would invert things like the Bayer pattern.
  // Instead we just leave everything flipped and correct it here by scaling by -1 on the y axis
  return (
    <mesh scale={[1, -1, 1]}>
      <planeBufferGeometry args={[geometry.width, geometry.height]} />
      <ImageMaterial image={image} />
    </mesh>
  );
};

const ImageMaterial = observer(({ image }: { image: Image }) => {
  const material = getImageMaterialType(image);
  const texture = getImageTexture(image);
  React.useEffect(() => {
    texture.needsUpdate = true;
    texture.flipY = false;
    texture.mapping = THREE.Texture.DEFAULT_MAPPING;
    texture.format = textureFormat(image);
    texture.type = THREE.UnsignedByteType;
    texture.wrapS = THREE.ClampToEdgeWrapping;
    texture.wrapT = THREE.ClampToEdgeWrapping;
    texture.magFilter = THREE.LinearFilter;
    texture.minFilter = THREE.LinearFilter;
    return () => texture.dispose();
  }, [texture]);
  if (material.type === "basic") {
    return <meshBasicMaterial map={texture} />;
  } else {
    return (
      <rawShaderMaterial
        vertexShader={vertexShader}
        fragmentShader={fragmentShader}
        depthTest={false}
        uniforms={{
          image: { value: texture },
          sourceSize: {
            value: [image.width, image.height, 1 / image.width, 1 / image.height],
          },
          firstRed: { value: material.firstRed },
          mosaicSize: { value: material.mosaicSize },
        }}
      />
    );
  }
});

function getImageTexture(image: Image): THREE.Texture {
  switch (image.type) {
    case "data":
      return new THREE.DataTexture(image.data.get(), image.width, image.height);
    case "element":
      return new THREE.Texture(image.element);
    case "bitmap":
      return new THREE.Texture(image.bitmap as any);
    default:
      throw new UnreachableError(image);
  }
}

function getImageMaterialType(image: Image) {
  switch (image.format) {
    case ImageFormat.JPEG:
    case ImageFormat.RGB8:
    case ImageFormat.GREY:
    case ImageFormat.GRAY:
    case ImageFormat.Y8__:
      return { type: "basic" };
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
      return { type: "bayer", firstRed: firstRed(image.format), mosaicSize: mosaicSize(image.format) };
    default:
      throw new UnreachableError(image.format);
  }
}

function textureFormat(image: Image): THREE.PixelFormat {
  switch (image.format) {
    case ImageFormat.JPEG:
    case ImageFormat.JPBG:
    case ImageFormat.JPRG:
    case ImageFormat.JPGR:
    case ImageFormat.JPGB:
    case ImageFormat.PJBG:
    case ImageFormat.PJRG:
    case ImageFormat.PJGR:
    case ImageFormat.PJGB:
      return THREE.RGBAFormat;
    case ImageFormat.RGB8:
      return THREE.RGBFormat;
    case ImageFormat.GRBG:
    case ImageFormat.RGGB:
    case ImageFormat.GBRG:
    case ImageFormat.BGGR:
    case ImageFormat.GREY:
    case ImageFormat.GRAY:
    case ImageFormat.Y8__:
      return THREE.RedFormat;
    default:
      throw new UnreachableError(image.format);
  }
}

function firstRed(format: BayerImageFormat): [number, number] {
  switch (format) {
    case ImageFormat.JPBG:
    case ImageFormat.BGGR:
    case ImageFormat.PJBG:
      return [1, 1];
    case ImageFormat.JPGR:
    case ImageFormat.GRBG:
    case ImageFormat.PJGR:
      return [1, 0];
    case ImageFormat.JPGB:
    case ImageFormat.GBRG:
    case ImageFormat.PJGB:
      return [0, 1];
    case ImageFormat.JPRG:
    case ImageFormat.RGGB:
    case ImageFormat.PJRG:
      return [0, 0];
    default:
      throw new UnreachableError(format);
  }
}

function mosaicSize(format: BayerImageFormat): number {
  switch (format) {
    case ImageFormat.BGGR:
    case ImageFormat.RGGB:
    case ImageFormat.GRBG:
    case ImageFormat.GBRG:
      return 1; // One value per width/height
    case ImageFormat.JPBG:
    case ImageFormat.JPRG:
    case ImageFormat.JPGR:
    case ImageFormat.JPGB:
      return 2; // Two values per width/height
    case ImageFormat.PJBG:
    case ImageFormat.PJRG:
    case ImageFormat.PJGR:
    case ImageFormat.PJGB:
      return 4; // Four values per width/height
    default:
      throw new UnreachableError(format);
  }
}
