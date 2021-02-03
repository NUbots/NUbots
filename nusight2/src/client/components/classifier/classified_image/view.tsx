import { computed } from 'mobx'
import { observer } from 'mobx-react'
import React from 'react'
import { Component } from 'react'

import { ObjectFit } from '../../three/three'
import { Three } from '../../three/three'

import { ClassifiedImageModel } from './model'
import { ClassifiedImageViewModel } from './view_model'

@observer
export class ClassifiedImageView extends Component<{ model: ClassifiedImageModel }> {
  render() {
    return <Three stage={this.stage} objectFit={this.objectFit} />
  }

  @computed.struct
  private get objectFit(): ObjectFit {
    const rawImage = this.props.model.rawImage
    if (rawImage) {
      return { type: 'contain', aspect: rawImage.image.height / rawImage.image.width }
    } else {
      return { type: 'fill' }
    }
  }

  private stage = () => {
    const viewModel = ClassifiedImageViewModel.of(this.props.model)
    return computed(() => viewModel.stage)
  }
}
