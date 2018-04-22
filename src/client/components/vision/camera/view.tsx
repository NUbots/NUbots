import { autorun } from 'mobx'
import { action } from 'mobx'
import { observer } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'

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
    const { width, height } = this.props.viewModel
    if (width == null || height == null) {
      return null
    }
    const aspectRatio = width / height
    const percentage = 60
    return <canvas
      style={{
        width: `${percentage}vw`,
        height: `${percentage / aspectRatio}vw`,
        maxHeight: `${percentage}vh`,
        maxWidth: `${percentage * aspectRatio}vh`,
      }}
      width={width}
      height={height}
      ref={this.onRef}
    />
  }

  @action
  private onRef = (canvas: HTMLCanvasElement | null) => {
    this.props.viewModel.canvas = canvas
  }

  private renderScene = () => {
    const { viewModel } = this.props
    const renderer = viewModel.renderer(viewModel.canvas)
    if (renderer) {
      renderer.render(viewModel.getScene(), viewModel.getCamera())
    }
  }
}
