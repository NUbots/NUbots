import React from "react";
import { useThree } from "@react-three/fiber";
import { observer } from "mobx-react";

import { Vector2 } from "../../../../../shared/math/vector2";
import { Vector3 } from "../../../../../shared/math/vector3";
import { Vector4 } from "../../../../../shared/math/vector4";
import { RawShaderMaterial } from "../../../three/three_fiber";
import { CameraModel } from "../../model";

import joinFragmentShader from "./shaders/join.frag";
import joinVertexShader from "./shaders/join.vert";
import segmentFragmentShader from "./shaders/segment.frag";
import segmentVertexShader from "./shaders/segment.vert";

export type Polyline = {
  points: PolylinePoint[];
  autoClose?: boolean;
  width?: number;
};

export interface PolylinePoint {
  pixel: Vector2;
  color: Vector4;
  size?: number;
}

export const PolylineView = React.memo(({ polyline, camera }: { polyline: Polyline; camera: CameraModel }) => (
  <PolylinesView polylines={[polyline]} camera={camera} />
));

export const PolylinesView = observer(({ polylines, camera }: { polylines: Polyline[]; camera: CameraModel }) => (
  <object3D>
    <Segments polylines={polylines} camera={camera} />
    <Joins polylines={polylines} camera={camera} />
  </object3D>
));

const SEGMENT_VERTICES = [[0, -0.5], [1, -0.5], [1, 0.5], [0, 0.5]].flat(); // prettier-ignore
const SEGMENT_UVS = [[0, 0], [1, 0], [1, 1], [0, 1]].flat(); // prettier-ignore
const INDEX = [0, 1, 2, 0, 2, 3];

const Segments = observer(({ polylines, camera }: { polylines: Polyline[]; camera: CameraModel }) => {
  const lineColors = ({ points, autoClose }: Polyline) =>
    points.concat(autoClose ? points[0] : []).flatMap((join) => join.color.toArray());

  const lineWidths = ({ points, width = 5, autoClose }: Polyline) =>
    new Array(autoClose ? points.length : points.length - 1).fill(width);

  const linePositions = ({ points, autoClose }: Polyline) =>
    points
      .flatMap((join) => Vector3.from(join.pixel).toArray())
      .concat(autoClose ? Vector3.from(points[0].pixel).toArray() : []);

  const colorStart = new Float32Array(polylines.flatMap((line) => lineColors(line).slice(0, -4)));
  const colorEnd = new Float32Array(polylines.flatMap((line) => lineColors(line).slice(4)));
  const width = new Float32Array(polylines.flatMap((line) => lineWidths(line)));
  const start = new Float32Array(polylines.flatMap((line) => linePositions(line).slice(0, -3)));
  const end = new Float32Array(polylines.flatMap((line) => linePositions(line).slice(3)));

  const { size: canvas } = useThree();
  return (
    <mesh frustumCulled={false}>
      <instancedBufferGeometry>
        <uint16BufferAttribute attach="index" args={[INDEX, 1]} />
        <float32BufferAttribute attach="attributes-position" args={[SEGMENT_VERTICES, 2]} />
        <float32BufferAttribute attach="attributes-uv" args={[SEGMENT_UVS, 2]} />
        <instancedBufferAttribute attach="attributes-startPoint" args={[start, 3]} />
        <instancedBufferAttribute attach="attributes-endPoint" args={[end, 3]} />
        <instancedBufferAttribute attach="attributes-startColor" args={[colorStart, 4]} />
        <instancedBufferAttribute attach="attributes-endColor" args={[colorEnd, 4]} />
        <instancedBufferAttribute attach="attributes-width" args={[width, 1]} />
      </instancedBufferGeometry>
      <RawShaderMaterial
        vertexShader={segmentVertexShader}
        fragmentShader={segmentFragmentShader}
        uniforms={{
          viewSize: { value: new Vector2(canvas.width, canvas.height).toThree() },
          imageSize: { value: new Vector2(camera.image.width, camera.image.height).toThree() },
          imageAspectRatio: { value: camera.image.width / camera.image.height },
        }}
        depthTest={false}
        depthWrite={false}
        transparent={true}
      />
    </mesh>
  );
});

const Joins = observer(({ polylines, camera }: { polylines: Polyline[]; camera: CameraModel }) => {
  const joins = polylines.flatMap((line) => line.points);
  const colors = new Float32Array(joins.flatMap((join) => join.color.toArray()));
  const radii = new Float32Array(joins.flatMap((join) => join.size ?? 10));
  const positions = new Float32Array(joins.flatMap((v) => v.pixel.toArray()));
  const { size: canvas } = useThree();
  return (
    <instancedMesh count={joins.length} frustumCulled={false}>
      <circleBufferGeometry args={[0.5, 12]}>
        <instancedBufferAttribute attach="attributes-lineJoin" args={[positions, 2]} />
        <instancedBufferAttribute attach="attributes-color" args={[colors, 4]} />
        <instancedBufferAttribute attach="attributes-radius" args={[radii, 1]} />
      </circleBufferGeometry>
      <RawShaderMaterial
        vertexShader={joinVertexShader}
        fragmentShader={joinFragmentShader}
        uniforms={{
          viewSize: { value: new Vector2(canvas.width, canvas.height).toThree() },
          imageSize: { value: new Vector2(camera.image.width, camera.image.height).toThree() },
          imageAspectRatio: { value: camera.image.width / camera.image.height },
        }}
        depthTest={false}
        depthWrite={false}
        transparent={true}
      />
    </instancedMesh>
  );
});
