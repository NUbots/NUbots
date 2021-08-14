import { computed } from 'mobx'
import { Points } from 'three'
import { BufferAttribute } from 'three'
import { BufferGeometry } from 'three'
import { LinearFilter } from 'three'
import { ClampToEdgeWrapping } from 'three'
import { UnsignedByteType } from 'three'
import { LuminanceFormat } from 'three'
import { Texture } from 'three'

import { disposableComputed } from '../../../base/disposable_computed'
import { Vector3 } from '../../../math/vector3'
import { shader } from '../../three/builders'
import { dataTexture } from '../../three/builders'
import { shaderMaterial } from '../../three/builders'
import { perspectiveCamera } from '../../three/builders'
import { scene } from '../../three/builders'
import { Stage } from '../../three/three'
import { Canvas } from '../../three/three'

import { VisualizerModel } from './model'
import fragmentShader from './shaders/visualizer.frag'
import vertexShader from './shaders/visualizer.vert'

export class VisualizerViewModel {
  private readonly canvas: Canvas
  private readonly model: VisualizerModel

  constructor(canvas: Canvas, model: VisualizerModel) {
    this.canvas = canvas
    this.model = model
  }

  static of(canvas: Canvas, model: VisualizerModel) {
    return new VisualizerViewModel(canvas, model)
  }

  @computed
  get stage(): Stage {
    return { camera: this.camera(), scene: this.scene() }
  }

  private readonly scene = scene(() => ({
    children: [this.points],
  }))

  private readonly camera = perspectiveCamera(() => ({
    fov: 75,
    aspect: this.canvas.width / this.canvas.height,
    near: 0.01,
    far: 15,
    up: Vector3.from({ x: 0, y: 0, z: 1 }),
    position: this.cameraPosition,
    lookAt: Vector3.of(),
  }))

  @computed
  private get cameraPosition() {
    const r = this.model.camera.distance
    const azimuth = this.model.camera.azimuth
    const elevation = this.model.camera.elevation
    return Vector3.from({
      x: r * Math.sin(Math.PI / 2 + elevation) * Math.cos(azimuth),
      y: r * Math.sin(Math.PI / 2 + elevation) * Math.sin(azimuth),
      z: r * Math.cos(Math.PI / 2 + elevation),
    })
  }

  @computed
  private get points(): Points {
    const points = new Points(this.pointsGeometry, this.planeMaterial())
    points.frustumCulled = false
    return points
  }

  @disposableComputed
  private get pointsGeometry(): BufferGeometry {
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

  private readonly planeMaterial = shaderMaterial(() => ({
    shader: this.shader,
    uniforms: {
      lut: { value: this.lutTexture() },
      lutSize: { value: this.lutSize },
      bitsX: { value: this.model.lut.size.x },
      bitsY: { value: this.model.lut.size.y },
      bitsZ: { value: this.model.lut.size.z },
      scale: { value: 1 },
      size: { value: this.canvas.height / this.model.lut.size.z / 7 },
    },
  }))

  private readonly shader = shader(() => ({ vertexShader, fragmentShader }))

  @computed
  get lutSize() {
    return Math.ceil(Math.sqrt(this.model.lut.data.length))
  }

  private readonly lutTexture = dataTexture(() => ({
    data: this.model.lut.data,
    width: this.lutSize,
    height: this.lutSize,
    format: LuminanceFormat,
    type: UnsignedByteType,
    mapping: Texture.DEFAULT_MAPPING,
    wrapS: ClampToEdgeWrapping,
    wrapT: ClampToEdgeWrapping,
    magFilter: LinearFilter,
    minFilter: LinearFilter,
    flipY: true,
  }))
}
