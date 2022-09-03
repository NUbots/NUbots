import { useThree } from '@react-three/fiber'
import { autorun } from 'mobx'
import { observer } from 'mobx-react'
import { useEffect } from 'react'
import { useMemo } from 'react'
import React from 'react'
import * as THREE from 'three'
import { Vector2 } from '../../../math/vector2'
import { CameraParams, VisualMesh } from './model'
import fragmentShader from './shaders/visual_mesh.frag'
import vertexShader from './shaders/visual_mesh.vert'

export const VisualMeshView = observer(
  ({ visualMesh, params }: { visualMesh: VisualMesh; params: CameraParams }) => {
    const geometry = useMemo(() => new THREE.BufferGeometry(), [])

    useEffect(() => {
      const dispose = autorun(() => {
        const { neighbours, rays, classifications } = visualMesh

        // Calculate our triangle indexes
        const nElem = rays.length / 3
        const triangles = []
        for (let i = 0; i < nElem; i++) {
          const ni = i * 6
          if (neighbours[ni + 0] < nElem) {
            if (neighbours[ni + 2] < nElem) {
              triangles.push(i, neighbours[ni + 0], neighbours[ni + 2])
            }
            if (neighbours[ni + 1] < nElem) {
              triangles.push(i, neighbours[ni + 1], neighbours[ni + 0])
            }
          }
        }

        const buffer = new THREE.InterleavedBuffer(
          new Float32Array(classifications.values.slice(0, -classifications.dim)),
          classifications.dim,
        )

        geometry.setIndex(triangles)
        const attributes = [
          { name: 'position', buffer: new THREE.Float32BufferAttribute(rays, 3) },
          { name: 'ball', buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 0) },
          { name: 'goal', buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 1) },
          { name: 'fieldLine', buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 2) },
          { name: 'field', buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 3) },
          { name: 'environment', buffer: new THREE.InterleavedBufferAttribute(buffer, 1, 4) },
        ]
        for (const [name] of Object.entries(geometry.attributes)) {
          geometry.removeAttribute(name)
        }
        for (const { name, buffer } of attributes) {
          geometry.addAttribute(name, buffer)
        }
      })
      return () => {
        dispose()
        geometry.dispose()
      }
    })

    const { size } = useThree()
    return (
      <mesh geometry={geometry}>
        <rawShaderMaterial
          vertexShader={vertexShader}
          fragmentShader={fragmentShader}
          uniforms={{
            Hcw: { value: params.Hcw.toThree() },
            viewSize: { value: new Vector2(size.width, size.height).toThree() },
            focalLength: { value: params.lens.focalLength },
            centre: { value: params.lens.centre.toThree() },
            k: { value: params.lens.distortionCoeffecients.toThree() },
            projection: { value: params.lens.projection },
          }}
          depthTest={false}
          depthWrite={false}
          transparent={true}
        />
      </mesh>
    )
  },
)
