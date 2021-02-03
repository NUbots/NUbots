import { computed } from 'mobx'
import { observer } from 'mobx-react'
import { Component } from 'react'
import React from 'react'

import { Canvas } from '../../three/three'
import { Three } from '../../three/three'

import { VisualizerController } from './controller'
import { VisualizerModel } from './model'
import styles from './styles.css'
import { VisualizerViewModel } from './view_model'

@observer
export class VisualizerView extends Component<{
  controller: VisualizerController
  model: VisualizerModel
}> {
  render() {
    return (
      <div className={styles.visualiser}>
        <Three
          stage={this.stage}
          onMouseDown={this.props.controller.onMouseDown}
          onMouseMove={this.props.controller.onMouseMove}
          onMouseUp={this.props.controller.onMouseUp}
          onWheel={this.props.controller.onWheel}
        />
      </div>
    )
  }

  private stage = (canvas: Canvas) => {
    const viewModel = VisualizerViewModel.of(canvas, this.props.model)
    return computed(() => viewModel.stage)
  }
}
