import { PlaneGeometry } from 'three'
import { PointLight } from 'three'
import { AmbientLight } from 'three'
import { BoxGeometry } from 'three'
import { Camera } from 'three'
import { LinearFilter } from 'three'
import { ClampToEdgeWrapping } from 'three'
import { UnsignedByteType } from 'three'
import { RGBAFormat } from 'three'
import { MultiplyOperation } from 'three'
import { Combine } from 'three'
import { WebGLRenderTarget } from 'three'
import { Texture } from 'three'
import { OrthographicCamera } from 'three'
import { PixelFormat } from 'three'
import { PerspectiveCamera } from 'three'
import { Scene } from 'three'
import { Geometry } from 'three'
import { TextureFilter } from 'three'
import { Wrapping } from 'three'
import { Mapping } from 'three'
import { TextureDataType } from 'three'
import * as THREE from 'three'
import { DataTexture } from 'three'
import { MeshPhongMaterial } from 'three'
import { MeshBasicMaterial } from 'three'
import { Color } from 'three'
import { Object3D } from 'three'
import { Mesh } from 'three'
import { Material } from 'three'
import { ShaderMaterial } from 'three'

import { Vector3 } from '../../math/vector3'

import { createUpdatableComputed } from './create_updatable_computed'

type Object3DOpts = {
  position?: Vector3,
  rotation?: Vector3,
  rotationOrder?: string,
  scale?: Vector3,
  up?: Vector3,
  children?: Object3D[]
}

export type StageOpts = {
  scene: Scene,
  camera: Camera,
  target?: WebGLRenderTarget
}

export const stage = createUpdatableComputed(
  ({ scene, camera, target }: StageOpts) => ({ scene, camera, target }),
  (stage, opts) => {
    stage.scene = opts.scene
    stage.camera = opts.camera
    stage.target = opts.target
  },
)

export const scene = createUpdatableComputed(
  (opts: Object3DOpts) => new Scene(),
  (scene, opts) => {
    scene.remove(...scene.children)
    opts.children && scene.add(...opts.children)
    updateObject3D(scene, opts)
  },
  scene => scene.dispose(),
)

export const group = createUpdatableComputed(
  (opts: Object3DOpts) => new Object3D(),
  (group, opts) => {
    group.remove(...group.children)
    opts.children && group.add(...opts.children)
    updateObject3D(group, opts)
  },
)

type PerspectiveCameraOpts = {
  fov: number,
  aspect: number,
  near: number,
  far: number,
  lookAt?: Vector3
}

export const perspectiveCamera = createUpdatableComputed(
  (opts: PerspectiveCameraOpts & Object3DOpts) => new PerspectiveCamera(),
  (camera, opts) => {
    camera.fov = opts.fov
    camera.aspect = opts.aspect
    camera.near = opts.near
    camera.far = opts.far
    camera.updateProjectionMatrix()
    updateObject3D(camera, opts)
    opts.lookAt && camera.lookAt(new THREE.Vector3(opts.lookAt.x, opts.lookAt.y, opts.lookAt.z))
  },
)

type OrthographicCameraOpts = {
  left: number,
  right: number,
  top: number,
  bottom: number,
  near: number,
  far: number
}

export const orthographicCamera = createUpdatableComputed(
  (opts: OrthographicCameraOpts & Object3DOpts) => new OrthographicCamera(opts.left, opts.right, opts.top, opts.bottom),
  (camera, opts) => {
    camera.left = opts.left
    camera.right = opts.right
    camera.top = opts.top
    camera.bottom = opts.bottom
    camera.near = opts.near
    camera.far = opts.far
    camera.updateProjectionMatrix()
    updateObject3D(camera, opts)
  },
)

type MeshOpts = {
  geometry: Geometry,
  material: Material | Material[]
}

export const mesh = createUpdatableComputed(
  (opts: MeshOpts & Object3DOpts) => new Mesh(opts.geometry, opts.material),
  (mesh, opts) => {
    mesh.geometry = opts.geometry
    mesh.material = opts.material
    updateObject3D(mesh, opts)
  },
)

type MeshBasicMaterialOpts = {
  color?: Color,
  map?: Texture,
  transparent?: boolean,
  combine?: Combine
}

const defaultColor = new Color(0xffffff)
const defaultCombine = MultiplyOperation
export const meshBasicMaterial = createUpdatableComputed(
  (opts: MeshBasicMaterialOpts) => new MeshBasicMaterial(),
  (material, opts) => {
    material.color = opts.color || defaultColor
    material.combine = opts.combine || defaultCombine
    material.map = opts.map || null
    material.transparent = opts.transparent != null ? opts.transparent : false
    material.needsUpdate = true
  },
  mesh => mesh.dispose(),
)

type MeshPhongMaterialOpts = {
  color?: Color,
  map?: Texture,
  transparent?: boolean
}

export const meshPhongMaterial = createUpdatableComputed(
  (opts: MeshPhongMaterialOpts) => new MeshPhongMaterial(),
  (material, opts) => {
    material.color = opts.color || defaultColor
    material.map = opts.map || null
    material.transparent = opts.transparent != null ? opts.transparent : false
    material.needsUpdate = true
  },
  material => material.dispose(),
)

type ShaderMaterialOpts = {
  vertexShader: string
  fragmentShader: string
  uniforms: { [uniform: string]: { value: any } }
}

export const shaderMaterial = createUpdatableComputed(
  (opts: ShaderMaterialOpts) => new ShaderMaterial(opts),
  (material, opts) => {
    material.vertexShader = opts.vertexShader
    material.fragmentShader = opts.fragmentShader
    for (const key of Object.keys(opts.uniforms)) {
      material.uniforms[key] = opts.uniforms[key]
    }
    material.needsUpdate = true
  },
  material => material.dispose(),
)

type TypedArray
  = Int8Array
  | Uint8Array
  | Uint8ClampedArray
  | Int16Array
  | Uint16Array
  | Int32Array
  | Uint32Array
  | Float32Array
  | Float64Array

type DataTextureOpts = {
  data: TypedArray,
  width: number,
  height: number,
  format: PixelFormat,
  type: TextureDataType,
  mapping: Mapping,
  wrapS: Wrapping,
  wrapT: Wrapping,
  magFilter: TextureFilter,
  minFilter: TextureFilter,
  flipY: boolean
}

export const dataTexture = createUpdatableComputed(
  (opts: DataTextureOpts) => new DataTexture(opts.data, opts.width, opts.height),
  (texture, opts) => {
    texture.format = opts.format
    texture.type = opts.type
    texture.mapping = opts.mapping
    texture.wrapS = opts.wrapS
    texture.wrapT = opts.wrapT
    texture.magFilter = opts.magFilter
    texture.minFilter = opts.minFilter
    texture.flipY = opts.flipY
    texture.needsUpdate = true
  },
  texture => texture.dispose(),
)

type TextureOpts = {
  format?: PixelFormat,
  type?: TextureDataType,
  wrapS?: Wrapping,
  wrapT?: Wrapping,
  magFilter?: TextureFilter,
  minFilter?: TextureFilter
}

type ImageTextureOpts = TextureOpts & {
  image?: HTMLImageElement,
  mapping: Mapping,
  flipY: boolean
}

export const imageTexture = createUpdatableComputed(
  // Unlike other builders in this file, `image` is optional.
  // This is because images need to be loaded, so it is often the case the data will not exist immediately.
  // Making it optional turns out to be very convenient as uniform values (such as textures) can be set asynchronously.
  (opts: ImageTextureOpts) => opts.image && new Texture(),
  (texture, opts) => {
    if (texture) {
      texture.image = opts.image
      texture.flipY = opts.flipY
      updateTexture(texture, opts)
      texture.needsUpdate = true
    }
  },
  texture => texture && texture.dispose(),
)

type RenderTargetOpts = TextureOpts & {
  width: number,
  height: number,
  anisotropy?: number;
  depthBuffer?: boolean;
  stencilBuffer?: boolean;
}

export const renderTarget = createUpdatableComputed(
  (opts: RenderTargetOpts) => new WebGLRenderTarget(opts.width, opts.height),
  (renderTarget, opts) => {
    renderTarget.setSize(opts.width, opts.height)
    renderTarget.texture.anisotropy = opts.anisotropy != null ? opts.anisotropy : 1
    renderTarget.depthBuffer = opts.depthBuffer != null ? opts.depthBuffer : true
    renderTarget.stencilBuffer = opts.stencilBuffer != null ? opts.stencilBuffer : true
    updateTexture(renderTarget.texture, opts)
  },
  renderTarget => renderTarget.dispose(),
)

function updateTexture(texture: Texture, opts: TextureOpts) {
  texture.format = opts.format || RGBAFormat
  texture.type = opts.type || UnsignedByteType
  texture.wrapS = opts.wrapS || ClampToEdgeWrapping
  texture.wrapT = opts.wrapT || ClampToEdgeWrapping
  texture.magFilter = opts.magFilter || LinearFilter
  texture.minFilter = opts.minFilter || LinearFilter
}

type BoxGeometryOpts = { width: number, height: number, depth: number }

export const boxGeometry = createUpdatableComputed(
  (opts: BoxGeometryOpts) => new BoxGeometry(opts.width, opts.height, opts.depth),
  (geometry, opts) => {
    const { width, height, depth } = geometry.parameters
    if (opts.width !== width || opts.height !== height || opts.depth !== depth) {
      geometry.copy(new BoxGeometry(opts.width, opts.height, opts.depth))
    }
  },
  box => box.dispose(),
)

type PlaneGeometryOpts = { width: number, height: number }

export const planeGeometry = createUpdatableComputed(
  (opts: PlaneGeometryOpts) => new PlaneGeometry(opts.width, opts.height),
  (geometry, opts) => {
    const { width, height } = geometry.parameters
    if (opts.width !== width || opts.height !== height) {
      geometry.copy(new PlaneGeometry(opts.width, opts.height))
    }
  },
  plane => plane.dispose(),
)

type LightOpts = Object3DOpts & { color?: Color, intensity?: number }

export const ambientLight = createUpdatableComputed(
  (opts: LightOpts) => new AmbientLight(opts.color, opts.intensity),
  (light, opts) => {
    light.color = opts.color != null ? opts.color : defaultColor
    light.intensity = opts.intensity != null ? opts.intensity : 1
    updateObject3D(light, opts)
  },
)

export const pointLight = createUpdatableComputed(
  (opts: LightOpts) => new PointLight(opts.color, opts.intensity),
  (light, opts) => {
    light.color = opts.color != null ? opts.color : defaultColor
    light.intensity = opts.intensity != null ? opts.intensity : 1
    updateObject3D(light, opts)
  },
)

function updateObject3D(object: Object3D, opts: Object3DOpts) {
  opts.position && object.position.set(opts.position.x, opts.position.y, opts.position.z)
  opts.rotation && object.rotation.set(opts.rotation.x, opts.rotation.y, opts.rotation.z, opts.rotationOrder)
  opts.scale && object.scale.set(opts.scale.x, opts.scale.y, opts.scale.z)
  opts.up && object.up.set(opts.up.x, opts.up.y, opts.up.z)
}
