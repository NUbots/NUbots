import { observer } from 'mobx-react'
import React from 'react'
import { Component } from 'react'

import { Renderer } from '../../../render2d/renderer'

import { FieldModel } from './model'
import style from './style.css'
import { FieldViewModel } from './view_model'

export type FieldProps = {
  model: FieldModel
}

// **** I need to figure out how I can update the state of the field robot and transfer these changes from one component into another
// **** I need to wrap mobx in a use context and then I can work with it in the display - https://www.youtube.com/watch?v=oQiMXRsO4o4

@observer
export class Field extends Component<FieldProps> {
  render() {
    const model = this.props.model
    const viewModel = FieldViewModel.of(model)
    return (
      <div className={style.container}>
        <Renderer
          engine="svg"
          className={style.field}
          scene={viewModel.scene}
          camera={viewModel.camera}
          aspectRatio={viewModel.aspectRatio}
        />
      </div>
    )
  }
}
