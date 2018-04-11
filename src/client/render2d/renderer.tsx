import * as React from 'react'

import { Transform } from '../math/transform'

import { CanvasRenderer } from './canvas_renderer'
import { Group } from './object/group'
import { PixiRenderer } from './pixi_renderer'
import { SVGRenderer } from './svg_renderer'

export type RendererProps = {
  className?: string
  scene: Group
  camera: Transform
  engine?: 'pixi' | 'svg' | 'canvas'
  aspectRatio?: number
}

export const Renderer = (props: RendererProps): JSX.Element => {
  switch (props.engine) {
    case undefined:
    case 'svg':
      return <SVGRenderer {...props} />
    case 'pixi':
      return <PixiRenderer {...props} />
    case 'canvas':
      return <CanvasRenderer {...props} />
    default:
      throw new Error(`Unknown rendering engine ${props.engine}`)
  }
}
