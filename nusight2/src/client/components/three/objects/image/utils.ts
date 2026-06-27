import { BayerImageFormat, Image, ImageFormat } from "@components/camera/image";
import { UnreachableError } from "@shared/base/unreachable_error";
import * as THREE from "three";

export function getImageData(image: Image) {
  switch (image.type) {
    case "bitmap":
      return image.bitmap;
    case "data":
      return image.data;
    case "element":
      return image.element;
  }
}

export function getTextureFormat(format: ImageFormat): THREE.PixelFormat {
  switch (format) {
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
      return THREE.LuminanceFormat;
    default:
      throw new UnreachableError(format);
  }
}

export function getFirstRed(format: BayerImageFormat): [number, number] {
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

export function getMosaicSize(format: BayerImageFormat): number {
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
