import { MathUtils } from "three";

import { Vector2 } from "./vector2";
import { Vector3 } from "./vector3";
import { Vector4 } from "./vector4";

/**
 * Typescript version of Lens Distortion Math
 */

/**
 * Projection types
 */
export enum Projection {
  UNKNOWN = 0,
  RECTILINEAR = 1,
  EQUIDISTANT = 2,
  EQUISOLID = 3,
}

/**
 * Calculates the image position of a unit vector projected onto the image with specific lens parameters.
 *
 * @param ray         Unit vector to project from the perspective of the camera
 * @param projection  Lens projection type
 * @param f           Normalised focal length of the lens in pixels / image width
 * @param center      Normalised center offset. Pixels from center to optical axis / image width
 * @param k           The polynomial distortion coefficients for the lens
 * @param dimensions  Pixel dimensions of the image
 *
 * @returns           Pixel position on the image of the projected ray
 */
export function project(
  ray: Vector3,
  projection: Projection,
  f: number,
  center: Vector2,
  k: Vector2,
  dimensions: Vector2,
) {
  const xVector = new Vector3(1, 0, 0);
  const theta = 2 * Math.atan2(ray.subtract(xVector).length, ray.add(xVector).length);
  const rSinTheta = 1 / Math.sqrt(1 - Math.pow(ray.x, 2));
  const rU = getR(projection, theta, f);
  const rD = distort(rU, k);
  const screen = ray.x >= 1 ? new Vector2(0, 0) : new Vector2(ray.y, ray.z).multiplyScalar(rD * rSinTheta);
  return dimensions.divideScalar(2).subtract(screen).subtract(center);
}

/**
 * Calculates the direction vector of a pixel in an image given the lens parameters of the camera.
 *
 * @param px          Pixel position on the image
 * @param projection  Lens projection type
 * @param f           Normalised focal length of the lens in pixels / image width
 * @param center      Normalised center offset. Pixels from center to optical axis / image width
 * @param k           The polynomial distortion coefficients for the lens
 * @param dimensions  Pixel dimensions of the image
 *
 * @returns           Direction vector of the given pixel in the image.
 */
export function unproject(
  px: Vector2,
  projection: Projection,
  f: number,
  center: Vector2,
  k: Vector2,
  dimensions: Vector2,
) {
  const screen = dimensions.divideScalar(2).subtract(px).subtract(center);
  const rD = screen.length;
  if (rD == 0) {
    return new Vector3(1, 0, 0);
  }

  const rU = undistort(rD, k);
  const theta = getTheta(projection, rU, f);
  const t = screen.multiplyScalar(Math.sin(theta)).divideScalar(rD);
  return new Vector3(Math.cos(theta), t.x, t.y);
}

function inverseCoefficients(k: Vector2) {
  const k2 = k.multiplyScalar(k.x, k.y); // k values squared
  return new Vector4(
    -k.x,
    3 * k2.x - k.y,
    -12 * k2.x * k.x + 8 * k.x * k.y,
    55 * k2.x * k2.x - 55 * k2.x * k.y + 5 * k2.y,
  );
}

/**
 * Takes an undistorted radial distance from the optical axis and computes and applies an inverse distortion.
 *
 * @param r The radius to distort
 * @param k The distortion coefficients
 *
 * @returns A distortion radius
 */
function distort(r: number, k: Vector2) {
  const ik = inverseCoefficients(k);
  return r * (1.0 + ik.x * Math.pow(r, 2) + ik.y * Math.pow(r, 4) + ik.z * Math.pow(r, 6) + ik.t * Math.pow(r, 8));
}

/**
 * Takes a distorted radial distance from the optical axis and applies the polynomial distortion coefficients to
 * undistort it.
 *
 * @param r The radius to undistort
 * @param k The undistortion coefficients
 *
 * @returns An undistorted radius
 */
function undistort(r: number, k: Vector2) {
  const r2 = Math.pow(r, 2);
  return r * (1.0 + k.x * r2 + k.y * Math.pow(r2, 2));
}

function equidistantR(theta: number, f: number) {
  return f * theta;
}

function equidistantTheta(r: number, f: number) {
  return r / f;
}

function rectilinearR(theta: number, f: number) {
  return f * Math.tan(MathUtils.clamp(theta, 0, Math.PI / 2));
}

function rectilinearTheta(r: number, f: number) {
  return Math.atan(r / f);
}

function equisolidR(theta: number, f: number) {
  return 2 * f * Math.sin(theta / 2);
}

function equisolidTheta(r: number, f: number) {
  return 2 * Math.asin(r / (2 * f));
}

function getR(projection: Projection, theta: number, f: number) {
  switch (projection) {
    case Projection.RECTILINEAR:
      return rectilinearR(theta, f);
    case Projection.EQUISOLID:
      return equisolidR(theta, f);
    case Projection.EQUIDISTANT:
      return equidistantR(theta, f);
    default:
      throw new Error(`Unknown Projection: ${projection}`);
  }
}

function getTheta(projection: Projection, r: number, f: number) {
  switch (projection) {
    case Projection.RECTILINEAR:
      return rectilinearTheta(r, f);
    case Projection.EQUISOLID:
      return equisolidTheta(r, f);
    case Projection.EQUIDISTANT:
      return equidistantTheta(r, f);
    default:
      throw new Error(`Unknown Projection: ${projection}`);
  }
}
