import * as classNames from 'classnames'
import { IReactionDisposer } from 'mobx'
import { observable } from 'mobx'
import { action } from 'mobx'
import { autorun } from 'mobx'
import { observer } from 'mobx-react'
import { WebGLRenderer } from 'pixi.js'
import { CanvasRenderer } from 'pixi.js'
import { Container } from 'pixi.js'
import { autoDetectRenderer } from 'pixi.js'
import * as React from 'react'
import { Component } from 'react'
import ReactResizeDetector from 'react-resize-detector'

import { Transform } from '../math/transform'

import { pixiObject } from './pixi/rendering'
import { RendererProps } from './renderer_props'
import * as style from './style.css'

@observer
export class PixiRenderer extends Component<RendererProps> {
  @observable private resolution: Transform = Transform.of()
  private canvas: HTMLCanvasElement | null = null
  private renderer: WebGLRenderer | CanvasRenderer
  private stopAutorun: IReactionDisposer

  componentDidMount() {

    this.renderer = autoDetectRenderer({
      view: this.canvas!,
      transparent: true,
      antialias: true,
    })

    // Render when changes happen
    this.stopAutorun = autorun(this.renderCanvas, {
      scheduler: requestAnimationFrame,
    })
  }

  componentWillUnmount() {
    if (this.stopAutorun) {
      this.stopAutorun()
    }
  }

  render() {
    return (
      <div className={classNames(this.props.className, style.container)}>
        <ReactResizeDetector handleWidth handleHeight onResize={this.onResize} />
        <canvas
          className={style.container}
          width={-this.resolution.translate.x * 2}
          height={-this.resolution.translate.y * 2}
          ref={this.onRef}
        />
      </div>
    )
  }

  private onRef = (canvas: HTMLCanvasElement | null) => {
    this.canvas = canvas
  }

  renderCanvas = () => {
    // Render our scene
    const { scene, camera } = this.props

    const transform = this.resolution.inverse().then(camera)

    const cam = new Container()
    cam.rotation = transform.rotate
    cam.x = transform.translate.x
    cam.y = transform.translate.y
    cam.scale.x = transform.scale.x
    cam.scale.y = transform.scale.y
    cam.addChild(pixiObject(this.props.scene))

    this.renderer.render(cam)
  }

  /**
   * This function is executed when the canvas resizes.
   * It is responsible for updating the viewport used by pixi
   * as well as adjusting the scale of the world to normalize it.
   * This means that when aspect ratio is set, the widest size will be [-1, 1]
   * After applying the camera transformation this means pixel coordinates don't need to be known.
   * Without aspect ratio the whole field of view will be [-1, 1] in both x and y (stretched view)
   */
  @action
  private onResize = (width: number, height: number) => {

    // Multiply all our widths by dpi
    width *= devicePixelRatio
    height *= devicePixelRatio

    // Update our renderer's viewport
    this.renderer.resize(width, height)

    // Translate to the center
    this.resolution.translate.x = -width * 0.5
    this.resolution.translate.y = -height * 0.5

    // If we have an aspect ratio, use it to scale the canvas to unit size
    if (this.props.aspectRatio !== undefined) {

      const canvasAspect = width / height
      const scale = canvasAspect < this.props.aspectRatio ? 1 / width : 1 / (height * this.props.aspectRatio)

      // Scale to fit
      this.resolution.scale.x = scale
      this.resolution.scale.y = scale
    } else {
      this.resolution.scale.x = 1 / width
      this.resolution.scale.y = 1 / height
    }
  }
}
