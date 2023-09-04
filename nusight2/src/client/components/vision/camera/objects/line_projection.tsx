import React from "react";
import { useThree } from "@react-three/fiber";
import { observer } from "mobx-react";
import * as THREE from "three";

import { Vector2 } from "../../../../../shared/math/vector2";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { mesh } from "../../../three/builders";
import { planeGeometry } from "../../../three/builders";
import { rawShader } from "../../../three/builders";
import { shaderMaterial } from "../../../three/builders";
import { Canvas } from "../../../three/three";
import { RawShaderMaterial } from "../../../three/three_fiber";
import { Lens } from "../lens";

import fragmentShader from "./shaders/line_projection.frag";
import vertexShader from "./shaders/line_projection.vert";

export interface ConeSegment {
  /** The camera space central axis of the cone. */
  axis: Vector3;
  /** The camera space vector pointing to the start of the segment. */
  start: Vector3;
  /** The camera space vector pointing to the end of the segment. */
  end: Vector3;
  /** The colour of the line to draw. */
  color?: Vector4;
  /** The width of the line to draw on the screen in pixels. */
  lineWidth?: number;
}

/** Draws a plane projected to infinity in world space. */
export const PlaneView = ({
  axis,
  color,
  lineWidth,
  lens,
  imageAspectRatio,
}: {
  axis: Vector3;
  color?: Vector4;
  lineWidth?: number;
  lens: Lens;
  imageAspectRatio: number;
}) => {
  // Pick an arbitrary orthogonal vector
  const start = Vector3.fromThree(
    !axis.x && !axis.y ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(-axis.y, axis.x, 0).normalize(),
  );
  return (
    <ConeSegmentView
      segment={{ axis, start, end: start, color, lineWidth }}
      lens={lens}
      imageAspectRatio={imageAspectRatio}
    />
  );
};
/** Draws a segment of a plane projected to infinity in world space. */
export const PlaneSegmentView = ({
  segment,
  lens,
  imageAspectRatio,
}: {
  segment: { axis?: Vector3; start: Vector3; end: Vector3; color?: Vector4; lineWidth?: number };
  lens: Lens;
  imageAspectRatio: number;
}) => {
  return (
    <ConeSegmentView
      segment={{
        ...segment,
        axis:
          segment.axis ??
          Vector3.fromThree(
            new THREE.Vector3().crossVectors(segment.start.toThree(), segment.end.toThree()).normalize(),
          ),
      }}
      lens={lens}
      imageAspectRatio={imageAspectRatio}
    />
  );
};

/** Draw a cone projected to infinity in world space. Only draws the positive cone, not the negative cone. */
export const ConeView = ({
  segment: { axis, radius, color, lineWidth },
  lens,
  imageAspectRatio,
}: {
  segment: { axis: Vector3; radius: number; color?: Vector4; lineWidth?: number };
  lens: Lens;
  imageAspectRatio: number;
}) => {
  // Pick an arbitrary orthogonal vector
  const orth = !axis.x && !axis.y ? new Vector3(0, 1, 0) : new Vector3(-axis.y, axis.x, 0).normalize();
  // Rotate our axis by this radius to get a start
  const start = Vector3.fromThree(axis.toThree().applyAxisAngle(orth.toThree(), Math.acos(radius)));
  return (
    <ConeSegmentView
      segment={{ axis, start, end: start, color, lineWidth }}
      lens={lens}
      imageAspectRatio={imageAspectRatio}
    />
  );
};

export const ConeSegmentView = observer(
  ({ segment, lens, imageAspectRatio }: { segment: ConeSegment; lens: Lens; imageAspectRatio: number }) => {
    const {
      size: { width, height },
    } = useThree();
    return (
      <mesh>
        <planeBufferGeometry args={[2, 2, 100]} />
        <RawShaderMaterial
          vertexShader={vertexShader}
          fragmentShader={fragmentShader}
          depthTest={false}
          depthWrite={false}
          transparent={true}
          uniforms={{
            viewSize: { value: new THREE.Vector2(width, height) },
            projection: { value: lens.projection },
            focalLength: { value: lens.focalLength },
            centre: { value: lens.centre.toThree() },
            k: { value: lens.distortionCoeffecients.toThree() },
            imageAspectRatio: { value: imageAspectRatio },
            axis: { value: segment.axis?.toThree() },
            start: { value: segment.start.toThree() },
            end: { value: segment.end.toThree() },
            color: { value: segment.color?.toThree() },
            lineWidth: { value: segment.lineWidth },
          }}
        />
      </mesh>
    );
  },
);
