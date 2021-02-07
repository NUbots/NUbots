import { storiesOf } from '@storybook/react'
import { observable } from 'mobx'
import { computed } from 'mobx'
import { action } from 'mobx'
import { Observer } from 'mobx-react'
import React from 'react'

import { fullscreen } from '../../../storybook/fullscreen'
import { ImageFormat } from '../../image'
import { Image } from '../../image'
import { ImageView } from '../view'

import jpegUrl from './images/image.jpg'
import jprgUrl from './images/image.jprg.jpg'
import rggbBinUrl from './images/image.rggb.bin'
import rggbUrl from './images/image.rggb.jpg'

const createModel = () => observable<{ image: Image | undefined }>({ image: undefined })

storiesOf('components.vision.image_view', module)
  .addDecorator(fullscreen)
  .add('JPEG', () => {
    const model = createModel()
    loadImageElement(jpegUrl, ImageFormat.JPEG).then(
      action((image: Image) => (model.image = image)),
    )
    return (
      <Observer>{() => <>{model.image ? <ImageView image={model.image} /> : null}</>}</Observer>
    )
  })
  .add('JPRG', () => {
    const model = createModel()
    loadImageElement(jprgUrl, ImageFormat.JPRG).then(
      action((image: Image) => (model.image = image)),
    )
    return (
      <Observer>{() => <>{model.image ? <ImageView image={model.image} /> : null}</>}</Observer>
    )
  })
  .add('RGGB', () => {
    const model = createModel()
    loadImageElement(rggbUrl, ImageFormat.RGGB).then(
      action((image: Image) => (model.image = image)),
    )
    return (
      <Observer>{() => <>{model.image ? <ImageView image={model.image} /> : null}</>}</Observer>
    )
  })
  .add('RGGB Buffer', () => {
    const model = createModel()
    loadImageData(rggbBinUrl, 600, 480, ImageFormat.RGGB).then(
      action((image: Image) => (model.image = image)),
    )
    return (
      <Observer>{() => <>{model.image ? <ImageView image={model.image} /> : null}</>}</Observer>
    )
  })

async function loadImageElement(url: string, format: ImageFormat): Promise<Image> {
  const element = await loadImage(url)
  const { width, height } = element
  return { type: 'element', width, height, element, format }
}

async function loadImageData(
  url: string,
  width: number,
  height: number,
  format: ImageFormat,
): Promise<Image> {
  const data = await fetchUrlAsBuffer(url)
  return { type: 'data', width, height, data: computed(() => new Uint8Array(data)), format }
}

async function loadImage(url: string): Promise<HTMLImageElement> {
  return new Promise((resolve, reject) => {
    const image = new Image()
    image.onload = () => resolve(image)
    image.onerror = () => reject()
    image.src = url
  })
}

async function fetchUrlAsBuffer(imageUrl: string): Promise<ArrayBuffer> {
  return fetch(imageUrl)
    .then(res => res.blob())
    .then(blob => {
      return new Promise<ArrayBuffer>((resolve, reject) => {
        const reader = new FileReader()
        reader.addEventListener('load', () => {
          const data = reader.result
          if (data && data instanceof ArrayBuffer) {
            resolve(data)
          } else {
            reject(reader.error)
          }
        })
        reader.readAsArrayBuffer(blob)
      })
    })
}
