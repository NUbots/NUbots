import { computed } from 'mobx'
import * as React from 'react'
import { Component } from 'react'

import { Canvas } from '../../three/three'
import { Three } from '../../three/three'

import { ClassifiedImageModel } from './model'
import { ClassifiedImageViewModel } from './view_model'

export class ClassifiedImageView extends Component<{ model: ClassifiedImageModel }> {
  render() {
    return <Three stage={this.stage}/>
  }

  private stage = (canvas: Canvas) => {
    const viewModel = ClassifiedImageViewModel.of(this.props.model)
    return computed(() => viewModel.stage)
  }
}
