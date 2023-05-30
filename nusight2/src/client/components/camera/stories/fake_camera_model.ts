import * as THREE from "three";

import { Matrix4 } from "../../../../shared/math/matrix4";
import { Projection } from "../../../../shared/math/projection";
import { Vector2 } from "../../../../shared/math/vector2";
import { CameraParams } from "../camera_params";
import type { Image } from "../image";
import { ImageFormat } from "../image";
import imageUrl from "../image_view/stories/images/image.jpg";
import { Lens } from "../lens";
import { CameraModel } from "../model";

/**
 * Create a fake version of the camera model to use for stories.
 */
export async function fakeCameraModel(): Promise<CameraModel> {
  const image = await loadImageElement(imageUrl, ImageFormat.JPEG);
  const Rcw = new THREE.Matrix4()
    .makeRotationZ((Math.PI * 13) / 32)
    .premultiply(new THREE.Matrix4().makeRotationY(-2 * Math.PI * (1.5 / 16)))
    .premultiply(new THREE.Matrix4().makeRotationX(Math.PI / 35));
  const rWCc = new THREE.Vector3(0, 10000, -6378137).applyMatrix4(Rcw);
  const Hcw = Matrix4.fromThree(Rcw.premultiply(new THREE.Matrix4().makeTranslation(rWCc.x, rWCc.y, rWCc.z)));
  const viewSize = Vector2.of(image.width, image.height);
  const focalLength = 193 / viewSize.x;
  return {
    name: "Fake Camera",
    image,
    params: new CameraParams({
      Hcw,
      lens: new Lens({
        projection: Projection.EQUIDISTANT,
        focalLength,
      }),
    }),
    drawOptions: {
      drawImage: true,
      drawDistance: false,
      drawCompass: true,
      drawHorizon: true,
    },
  };
}

async function loadImageElement(url: string, format: ImageFormat): Promise<Image> {
  const element = await loadImage(url);
  const { width, height } = element;
  return { type: "element", width, height, element, format };
}

async function loadImage(url: string): Promise<HTMLImageElement> {
  return new Promise((resolve, reject) => {
    const image = new Image();
    image.onload = () => resolve(image);
    image.onerror = () => reject();
    image.src = url;
  });
}
