import { computed } from 'mobx'
import { expr } from 'mobx-utils'
import { Vector3 } from 'three'
import { Points } from 'three'
import { BufferAttribute } from 'three'
import { BufferGeometry } from 'three'
import { PerspectiveCamera } from 'three'
import { Scene } from 'three'
import { Material } from 'three'
import { LinearFilter } from 'three'
import { ClampToEdgeWrapping } from 'three'
import { UnsignedByteType } from 'three'
import { LuminanceFormat } from 'three'
import { DataTexture } from 'three'
import { Texture } from 'three'
import { ShaderMaterial } from 'three'
import { Camera } from 'three'

import { disposableComputed } from '../../../base/disposable_computed'
import { Canvas } from '../../three/three'

import { VisualizerModel } from './model'
import * as fragmentShader from './shaders/visualizer.frag'
import * as vertexShader from './shaders/visualizer.vert'

export class VisualizerViewModel {
  constructor(private readonly canvas: Canvas, private readonly model: VisualizerModel) {
  }

  static of(canvas: Canvas, model: VisualizerModel) {
    return new VisualizerViewModel(canvas, model)
  }

  @computed
  get scene(): Scene {
    const scene = new Scene()
    scene.add(this.points)
    return scene
  }

  @computed
  get camera(): Camera {
    const aspect = expr(() => this.canvas.width / this.canvas.height)
    const camera = new PerspectiveCamera(75, aspect, 0.01, 15)
    camera.up.set(0, 0, 1)
    const r = this.model.camera.distance
    const azimuth = this.model.camera.azimuth
    const elevation = this.model.camera.elevation
    const x = r * Math.sin(Math.PI / 2 + elevation) * Math.cos(azimuth)
    const y = r * Math.sin(Math.PI / 2 + elevation) * Math.sin(azimuth)
    const z = r * Math.cos(Math.PI / 2 + elevation)
    camera.position.set(x, y, z)
    camera.lookAt(new Vector3(0, 0, 0))
    return camera
  }

  @computed
  get points(): Points {
    const points = new Points(this.pointsGeometry, this.planeMaterial)
    points.frustumCulled = false
    return points
  }

  @disposableComputed
  get pointsGeometry(): BufferGeometry {
    const lutSize = this.model.lut.data.length
    const geometry = new BufferGeometry()
    const vertices = new Float32Array(lutSize * 3)
    let index = 0
    const maxX = 2 ** this.model.lut.size.x
    const maxY = 2 ** this.model.lut.size.y
    const maxZ = 2 ** this.model.lut.size.z
    for (let r = 0; r < maxX; r++) {
      for (let g = 0; g < maxY; g++) {
        for (let b = 0; b < maxZ; b++) {
          vertices[index] = r
          vertices[index + 1] = g
          vertices[index + 2] = b
          index += 3
        }
      }
    }
    geometry.addAttribute('position', new BufferAttribute(vertices, 3))
    return geometry
  }

  @disposableComputed
  get planeMaterial(): Material {
    return new ShaderMaterial({
      vertexShader: String(vertexShader),
      fragmentShader: String(fragmentShader),
      uniforms: {
        lut: { value: this.lutTexture },
        lutSize: { value: this.lutSize },
        bitsX: { value: this.model.lut.size.x },
        bitsY: { value: this.model.lut.size.y },
        bitsZ: { value: this.model.lut.size.z },
        scale: { value: 1 },
        size: { value: this.canvas.height / this.model.lut.size.z / 7 },
      },
    })
  }

  @computed
  get lutSize() {
    return Math.ceil(Math.sqrt(this.model.lut.data.length))
  }

  @disposableComputed
  get lutTexture(): Texture {
    const texture = new DataTexture(
      this.model.lut.data,
      this.lutSize,
      this.lutSize,
      LuminanceFormat,
      UnsignedByteType,
      Texture.DEFAULT_MAPPING,
      ClampToEdgeWrapping,
      ClampToEdgeWrapping,
      LinearFilter,
      LinearFilter,
    )
    texture.flipY = true
    texture.needsUpdate = true
    return texture
  }
}
