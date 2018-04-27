import * as classNames from 'classnames'
import { observable } from 'mobx'
import { action } from 'mobx'
import { observer } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'
import ReactResizeDetector from 'react-resize-detector'

import { Transform } from '../math/transform'

import { RendererProps } from './renderer_props'
import * as style from './style.css'
import { Group } from './svg/group'
import { toSvgTransform } from './svg/rendering'

@observer
export class SVGRenderer extends Component<RendererProps> {
  @observable private resolution: Transform = Transform.of()

  render() {
    const { className, scene, camera } = this.props

    const cam = this.resolution.inverse().then(camera)
    return (
      <div className={classNames(className, style.container)}>
        <ReactResizeDetector handleWidth handleHeight onResize={this.onResize} />
        <svg className={style.container}>
          <g transform={toSvgTransform(cam)}>
            <Group model={scene} world={cam}/>
          </g>
        </svg>
      </div>
    )
  }

  @action
  private onResize = (width: number, height: number) => {

    // Translate to the center
    this.resolution.translate.x = -width * 0.5
    this.resolution.translate.y = -height * 0.5

    // If we have an aspect ratio, use it to scale the canvas to unit size
    if (this.props.aspectRatio !== undefined) {

      const canvasAspect = width / height
      const scale = canvasAspect < this.props.aspectRatio ? 1 / width : 1 / (height * this.props.aspectRatio)

      // Scale to fit
      this.resolution.scale.x = scale
      this.resolution.scale.y = -scale
    } else {
      this.resolution.scale.x = 1 / width
      this.resolution.scale.y = -1 / height
    }
  }
}
