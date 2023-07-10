import { IComputedValue } from "mobx";

export type Image = DataImage | ElementImage | BitmapImage;

interface BaseImage {
  width: number;
  height: number;
  format: ImageFormat;
}

export interface DataImage extends BaseImage {
  type: "data";
  data: IComputedValue<Uint8Array>;
}

export interface BitmapImage extends BaseImage {
  type: "bitmap";
  bitmap: ImageBitmap;
}

export interface ElementImage extends BaseImage {
  type: "element";
  element: HTMLImageElement;
}

export enum ImageFormat {
  JPEG, // JPEG image
  BGGR, // color bayer
  RGGB, // color bayer
  GRBG, // color bayer
  GBRG, // color bayer
  JPBG, // Permuted color bayer
  JPRG, // Permuted color bayer
  JPGR, // Permuted color bayer
  JPGB, // Permuted color bayer
  PJRG, // Permuted polarized color bayer
  PJGR, // Permuted polarized color bayer
  PJGB, // Permuted polarized color bayer
  PJBG, // Permuted polarized monochrome bayer
  RGB8, // Raw RGB color
  GREY, // Monochrome
  GRAY, // Monochrome
  Y8__, // Monochrome
}

export type BayerImageFormat =
  | ImageFormat.BGGR
  | ImageFormat.RGGB
  | ImageFormat.GRBG
  | ImageFormat.GBRG
  | ImageFormat.JPBG
  | ImageFormat.JPRG
  | ImageFormat.JPGR
  | ImageFormat.PJRG
  | ImageFormat.PJGR
  | ImageFormat.PJGB
  | ImageFormat.PJBG
  | ImageFormat.JPGB;
