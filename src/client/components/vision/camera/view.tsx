import { autorun } from 'mobx'
import { action } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'
import { Component } from 'react'
import ReactResizeDetector from 'react-resize-detector'

import { SwitchesMenu } from '../../switches_menu/view'

import { CameraViewModel } from './view_model'

@observer
export class CameraView extends Component<{ viewModel: CameraViewModel }> {

  private destroy: () => void = () => {
  }

  componentDidMount() {
    this.destroy = autorun(this.renderScene, { scheduler: requestAnimationFrame })
  }

  componentWillUnmount() {
    this.destroy()
  }

  render() {
    const { imageWidth, imageHeight, drawOptions } = this.props.viewModel

    if (!imageWidth || !imageHeight) {
      return null
    }

    const aspectRatio = imageWidth / imageHeight
    const percentage = 60

    return (
      <div
        style={{
          width: `${percentage}vw`,
          height: `${percentage / aspectRatio}vw`,
          maxHeight: `${percentage}vh`,
          maxWidth: `${percentage * aspectRatio}vh`,
          position: 'relative',
        }}>
        <ReactResizeDetector handleWidth handleHeight onResize={this.onResize}/>
        <canvas ref={this.onRef}/>
        <div style={{ position: 'absolute', top: '0', right: '0' }}>
          <SwitchesMenu
            dropdownMenuPosition='right'
            options={drawOptions}
          />
        </div>
      </div>
    )
  }

  @action
  private onRef = (canvas: HTMLCanvasElement | null) => {
    this.props.viewModel.canvas = canvas
  }

  @action
  private onResize = (width: number, height: number) => {

    const { renderer, canvas } = this.props.viewModel
    canvas && renderer(canvas)!.setSize(width, height)
    this.props.viewModel.viewWidth = width
    this.props.viewModel.viewHeight = height
  }

  private renderScene = () => {
    const { viewModel } = this.props
    const renderer = viewModel.renderer(viewModel.canvas)
    if (renderer) {
      renderer.render(viewModel.getScene(), viewModel.camera)
    }
  }
}
