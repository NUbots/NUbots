import { CameraParams } from "./camera_params";
import { Image } from "./image";

export interface CameraDefaultDrawOptions {
  drawImage: boolean;
  drawDistance: boolean;
  drawCompass: boolean;
  drawHorizon: boolean;
}

export interface CameraModel {
  name: string;
  image: Image;
  params: CameraParams;
  drawOptions: CameraDefaultDrawOptions;
}
