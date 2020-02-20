import { computed } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'
import { Component } from 'react'

import { ObjectFit } from '../../three/three'
import { Canvas } from '../../three/three'
import { Three } from '../../three/three'

import { CameraModel } from './model'
import { CameraViewModel } from './view_model'

@observer
export class CameraView extends Component<{ model: CameraModel }> {
  render() {
    return <Three stage={this.stage} objectFit={this.objectFit}/>
  }

  @computed.struct
  private get objectFit(): ObjectFit {
    const { width, height } = this.props.model.image
    return { type: 'contain', aspect: height / width }
  }

  private readonly stage = (canvas: Canvas) => {
    const cameraViewModel = CameraViewModel.of(canvas, this.props.model)
    return computed(() => [cameraViewModel.stage])
  }
}
