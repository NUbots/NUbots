import { Canvas } from '@react-three/fiber'
import { autorun } from 'mobx'
import { useEffect } from 'react'
import * as THREE from 'three'
import { observer } from 'mobx-react'
import { useMemo } from 'react'
import React from 'react'
import { UnreachableError } from '../../../../shared/base/unreachable_error'

import { BayerImageFormat } from '../image'
import { ImageFormat } from '../image'
import { Image } from '../image'

import fragmentShader from './shaders/bayer.frag'
import vertexShader from './shaders/bayer.vert'

export const ImageView = observer(({ image }: { image: Image }) => {
  const camera = useMemo(() => {
    const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -1, 1)
    ;(camera as any).manual = true
    return camera
  }, [])
  return (
    <Canvas
      gl={{ antialias: false, alpha: false, depth: false, stencil: false }}
      orthographic={true}
      camera={camera}
      linear={true}
      flat={true}
      style={{ backgroundColor: 'black' }}
    >
      <ImageMesh image={image} />
    </Canvas>
  )
})

export const ImageMesh = observer(({ image }: { image: Image }) => {
  // Normally this effect could be achieved by setting texture.flipY to make
  // the textures the correct way up again. However this is ignored on RenderTargets
  // We can't flip it at the raw stage either as this would invert things like the Bayer pattern.
  // Instead we just leave everything flipped and correct it here by scaling by -1 on the y axis
  return (
    <mesh scale={[1, -1, 1]}>
      <planeGeometry args={[2, 2]} />
      <Material image={image} />
    </mesh>
  )
})

const Material = observer(({ image }: { image: Image }) => {
  if (isBayerImage(image)) {
    return <BayerMaterial image={image} />
  }
  const texture = useImageTexture(image)
  return <meshBasicMaterial map={texture} />
})

const BayerMaterial = observer(({ image }: { image: Image & { format: BayerImageFormat } }) => {
  const texture = useImageTexture(image)
  return (
    <rawShaderMaterial
      vertexShader={vertexShader}
      fragmentShader={fragmentShader}
      depthTest={false}
      uniforms={{
        image: { value: texture },
        sourceSize: {
          value: [image.width, image.height, 1 / image.width, 1 / image.height],
        },
        firstRed: { value: firstRed(image.format) },
        mosaicSize: { value: mosaicSize(image.format) },
      }}
    />
  )
})

function useImageTexture(image: Image) {
  const texture = useMemo(() => {
    switch (image.type) {
      case 'element':
        return new THREE.Texture()
      case 'data':
        return new THREE.DataTexture(image.data.get(), image.width, image.height)
      default:
        throw new UnreachableError(image)
    }
  }, [image.type])

  useEffect(() =>
    autorun(() => {
      switch (image.type) {
        case 'element':
          texture.image = image.element
          break
        case 'data':
          texture.image = {
            data: image.data.get(),
            width: image.width,
            height: image.height,
          }
          break
      }
      texture.minFilter = THREE.LinearFilter
      texture.magFilter = THREE.LinearFilter
      texture.encoding = THREE.LinearEncoding
      texture.format = textureFormat(image.format)
      texture.flipY = false
      texture.needsUpdate = true
    }),
  )

  return texture
}

function isBayerImage(image: Image): image is Image & { format: BayerImageFormat } {
  return isBayerFormat(image.format)
}

function isBayerFormat(format: ImageFormat): format is BayerImageFormat {
  switch (format) {
    case ImageFormat.JPEG:
    case ImageFormat.RGB8:
    case ImageFormat.GREY:
    case ImageFormat.GRAY:
    case ImageFormat.Y8__:
      return false
    case ImageFormat.JPBG:
    case ImageFormat.JPRG:
    case ImageFormat.JPGR:
    case ImageFormat.JPGB:
    case ImageFormat.GRBG:
    case ImageFormat.RGGB:
    case ImageFormat.GBRG:
    case ImageFormat.BGGR:
    case ImageFormat.PJBG:
    case ImageFormat.PJRG:
    case ImageFormat.PJGR:
    case ImageFormat.PJGB:
      return true
    default:
      throw new UnreachableError(format)
  }
}

function textureFormat(format: ImageFormat): THREE.PixelFormat {
  switch (format) {
    case ImageFormat.JPEG:
    case ImageFormat.JPBG:
    case ImageFormat.JPRG:
    case ImageFormat.JPGR:
    case ImageFormat.JPGB:
    case ImageFormat.PJBG:
    case ImageFormat.PJRG:
    case ImageFormat.PJGR:
    case ImageFormat.PJGB:
      return THREE.RGBAFormat
    case ImageFormat.RGB8:
      return THREE.RGBFormat
    case ImageFormat.GRBG:
    case ImageFormat.RGGB:
    case ImageFormat.GBRG:
    case ImageFormat.BGGR:
    case ImageFormat.GREY:
    case ImageFormat.GRAY:
    case ImageFormat.Y8__:
      return THREE.LuminanceFormat
    default:
      throw new UnreachableError(format)
  }
}

function firstRed(format: BayerImageFormat): [number, number] {
  switch (format) {
    case ImageFormat.JPBG:
    case ImageFormat.BGGR:
    case ImageFormat.PJBG:
      return [1, 1]
    case ImageFormat.JPGR:
    case ImageFormat.GRBG:
    case ImageFormat.PJGR:
      return [1, 0]
    case ImageFormat.JPGB:
    case ImageFormat.GBRG:
    case ImageFormat.PJGB:
      return [0, 1]
    case ImageFormat.JPRG:
    case ImageFormat.RGGB:
    case ImageFormat.PJRG:
      return [0, 0]
    default:
      throw new UnreachableError(format)
  }
}

function mosaicSize(format: BayerImageFormat): number {
  switch (format) {
    case ImageFormat.BGGR:
    case ImageFormat.RGGB:
    case ImageFormat.GRBG:
    case ImageFormat.GBRG:
      return 1 // One value per width/height
    case ImageFormat.JPBG:
    case ImageFormat.JPRG:
    case ImageFormat.JPGR:
    case ImageFormat.JPGB:
      return 2 // Two values per width/height
    case ImageFormat.PJBG:
    case ImageFormat.PJRG:
    case ImageFormat.PJGR:
    case ImageFormat.PJGB:
      return 4 // Four values per width/height
    default:
      throw new UnreachableError(format)
  }
}
