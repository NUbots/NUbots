import { Object3DNode } from '@react-three/fiber'
import { extend } from '@react-three/fiber'
import { observer } from 'mobx-react'
import React from "react";
import { useThree } from "@react-three/fiber";
import * as THREE from "three";
import { InterleavedBufferAttribute } from "three";

import { Vector2 } from "../../../../../shared/math/vector2";
import { CameraParams } from "../camera_params";

import fragmentShader from "./shaders/visual_mesh.frag";
import vertexShader from "./shaders/visual_mesh.vert";
import { RawShaderMaterial } from '../../../three/three_fiber';

export interface VisualMeshModel {
  readonly neighbours: number[];
  readonly rays: number[];
  readonly classifications: { dim: number; values: number[] };
}

export const VisualMeshView = observer(({
  model,
  params,
  imageAspectRatio,
}: {
  model: VisualMeshModel;
  params: CameraParams;
  imageAspectRatio: number;
}) => {
  const { size } = useThree();
  if (!model.rays.length) {
    return null;
  }
  const { index, attributes } = React.useMemo(() => {
    const { neighbours, rays, classifications } = model;

    // Calculate our triangle indexes
    const nElem = rays.length / 3;
    const triangles = [];
    for (let i = 0; i < nElem; i++) {
      const ni = i * 6;

      if (neighbours[ni + 0] < nElem && neighbours[ni + 1] < nElem) {
        triangles.push(i, neighbours[ni + 0], neighbours[ni + 1]);
        triangles.push(i, neighbours[ni + 1], neighbours[ni + 0]);
      }
      if (neighbours[ni + 1] < nElem && neighbours[ni + 2] < nElem) {
        triangles.push(i, neighbours[ni + 1], neighbours[ni + 2]);
        triangles.push(i, neighbours[ni + 2], neighbours[ni + 1]);
      }
    }
    const buffer = new THREE.InterleavedBuffer(
        new Float32Array(classifications.values.slice(0, -classifications.dim)),
        classifications.dim,
    );

    return {
      index: triangles,
      attributes: {
        position: rays,
        buffer,
      },
    };
  }, [model.neighbours, model.rays, model.classifications]);

  return (
    <mesh>
      <bufferGeometry>
        <uint16BufferAttribute attach="index" args={[index, 1]}/>
        <float32BufferAttribute attach="attributes-position" args={[attributes.position, 3]}/>
        <interleavedBufferAttribute attach="attributes-ball" args={[attributes.buffer, 1, 0]}/>
        <interleavedBufferAttribute attach="attributes-goal" args={[attributes.buffer, 1, 1]}/>
        <interleavedBufferAttribute attach="attributes-fieldLine" args={[attributes.buffer, 1, 2]}/>
        <interleavedBufferAttribute attach="attributes-field" args={[attributes.buffer, 1, 3]}/>
        <interleavedBufferAttribute attach="attributes-environment" args={[attributes.buffer, 1, 4]}/>
      </bufferGeometry>
      <RawShaderMaterial
        vertexShader={vertexShader}
        fragmentShader={fragmentShader}
        depthTest={false}
        depthWrite={false}
        transparent={true}
        wireframe={true}
        uniforms={{
          Hcw: { value: params.Hcw.toThree() },
          viewSize: { value: new Vector2(size.width, size.height).toThree() },
          focalLength: { value: params.lens.focalLength },
          centre: { value: params.lens.centre.toThree() },
          k: { value: params.lens.distortionCoeffecients.toThree() },
          imageAspectRatio: { value: imageAspectRatio },
          projection: { value: params.lens.projection },
        }}
      />
    </mesh>
  );
});

// Extends the JSX namespace to include the <interleavedBufferAttribute/> element.
// See: https://docs.pmnd.rs/react-three-fiber/tutorials/typescript#extending-threeelements
// TODO (Annable): Remove once R3F types contains InterleavedBufferAttribute
// See: https://github.com/pmndrs/react-three-fiber/blob/master/packages/fiber/src/three-types.ts
declare module '@react-three/fiber' {
  interface ThreeElements {
    interleavedBufferAttribute: Object3DNode<THREE.InterleavedBufferAttribute, typeof THREE.InterleavedBufferAttribute>
  }
}

extend({ InterleavedBufferAttribute });
