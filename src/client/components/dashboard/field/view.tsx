import { observer } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'

import { CanvasRenderer } from '../../../render2d/canvas_renderer'
import { SVGRenderer } from '../../../render2d/svg_renderer'

import { FieldModel } from './model'
import * as style from './style.css'
import { FieldViewModel } from './view_model'

export type FieldProps = {
  model: FieldModel
}

@observer
export class Field extends Component<FieldProps> {

  render() {
    const model = this.props.model
    const viewModel = FieldViewModel.of(model)
    return <div className={style.container}>
      <SVGRenderer
        className={style.field}
        scene={viewModel.scene}
        camera={viewModel.camera}
        aspectRatio={viewModel.aspectRatio} />
    </div>
  }
}
