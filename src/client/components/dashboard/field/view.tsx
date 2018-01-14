import { action } from 'mobx'
import { autorun } from 'mobx'
import { IReactionDisposer } from 'mobx'
import { observer } from 'mobx-react'
import * as React from 'react'
import { Component } from 'react'

import { CanvasRenderer } from '../../../canvas/renderer'

import { FieldController } from './controller'
import { FieldModel } from './model'
import * as style from './style.css'
import { FieldViewModel } from './view_model'

export type FieldProps = {
  controller: FieldController
  model: FieldModel
}

@observer
export class Field extends Component<FieldProps> {
  private field: HTMLCanvasElement
  private rafId: number
  private renderer: CanvasRenderer
  private stopAutorun: IReactionDisposer

  componentDidMount() {
    if (!this.field) {
      return
    }
    const context: CanvasRenderingContext2D = this.field.getContext('2d')!
    this.renderer = CanvasRenderer.of(context)
    this.stopAutorun = autorun(() => this.renderField())
    this.rafId = requestAnimationFrame(this.onAnimationFrame)
  }

  componentWillUnmount() {
    cancelAnimationFrame(this.rafId)
  }

  render() {
    return <canvas className={style.field} ref={this.onRef}/>
  }

  private onAnimationFrame = () => {
    this.rafId = requestAnimationFrame(this.onAnimationFrame)
    const width = this.field.clientWidth
    const height = this.field.clientHeight
    if (width !== this.field.width || height !== this.field.height) {
      this.onFieldResize(width, height)
    }
  }

  private onRef = (field: HTMLCanvasElement) => {
    this.field = field
  }

  private renderField(): void {
    const viewModel = FieldViewModel.of(this.props.model)
    this.renderer.render(viewModel.scene, this.props.model.camera)
  }

  @action
  private onFieldResize(width: number, height: number) {
    this.field.width = width
    this.field.height = height
    this.props.controller.onFieldResize(this.props.model, width, height)
  }
}
