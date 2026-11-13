import React, { useEffect, useMemo } from "react";
import * as THREE from "three";

import { Matrix4 } from "../../../../../shared/math/matrix4";
import { CameraImage } from "../../robot_model";

import fragmentShader from "./shaders/camera_projection.frag";
import vertexShader from "./shaders/camera_projection.vert";

interface CameraProjectionProps {
  /** The camera image, with the lens and world to camera transform it was captured with */
  image: CameraImage;
  /** World to field transform */
  Hfw: Matrix4;
  /** Points on the field further than this from the camera are not drawn (metres) */
  maxDistance?: number;
  opacity?: number;
}

/**
 * Projects a camera image out onto the field plane, using the camera's lens model and its pose on the field.
 *
 * Each fragment of the ground plane is transformed into camera space and projected through the lens model to find
 * which pixel of the image (if any) sees that point on the field. If the camera intrinsics, extrinsics and the
 * localisation are all correct, the field lines in the image will sit on top of the field lines of the field model.
 */
export const CameraProjection: React.FC<CameraProjectionProps> = ({ image, Hfw, maxDistance = 12, opacity = 0.85 }) => {
  // A plane big enough to cover everything within maxDistance of anywhere on the field. Fragments the camera
  // cannot see are discarded in the shader.
  const geometry = useMemo(() => new THREE.PlaneGeometry(40, 40), []);

  const texture = useMemo(() => {
    const texture = new THREE.Texture(image.bitmap);
    // Image bitmaps cannot be flipped on upload, the shader's texture coordinates account for this
    texture.flipY = false;
    texture.generateMipmaps = false;
    texture.minFilter = THREE.LinearFilter;
    texture.magFilter = THREE.LinearFilter;
    texture.needsUpdate = true;
    return texture;
  }, [image.bitmap]);
  useEffect(() => () => texture.dispose(), [texture]);

  const uniforms = useMemo(
    () => ({
      image: { value: null as THREE.Texture | null },
      imageAspectRatio: { value: 1 },
      Hcf: { value: new THREE.Matrix4() },
      focalLength: { value: 1 },
      projection: { value: 0 },
      centre: { value: new THREE.Vector2() },
      k: { value: new THREE.Vector2() },
      maxDistance: { value: maxDistance },
      opacity: { value: opacity },
    }),
    [],
  );

  // Hcf = Hcw * Hwf, using the Hcw the image was captured with
  uniforms.image.value = texture;
  uniforms.imageAspectRatio.value = image.width / image.height;
  uniforms.Hcf.value.copy(image.Hcw.toThree()).multiply(Hfw.toThree().invert());
  uniforms.focalLength.value = image.lens.focalLength;
  uniforms.projection.value = image.lens.projection;
  uniforms.centre.value.set(image.lens.centre.x, image.lens.centre.y);
  uniforms.k.value.set(image.lens.distortionCoefficients.x, image.lens.distortionCoefficients.y);
  uniforms.maxDistance.value = maxDistance;
  uniforms.opacity.value = opacity;

  return (
    // Sits just above the field so that it draws over the grass and field lines of the field model
    <mesh geometry={geometry} position={[0, 0, 0.003]} renderOrder={1}>
      <rawShaderMaterial
        vertexShader={vertexShader}
        fragmentShader={fragmentShader}
        uniforms={uniforms}
        transparent
        depthWrite={false}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
};
