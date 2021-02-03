import { observer } from 'mobx-react'
import { ChangeEvent } from 'react'
import { ComponentType } from 'react'
import React from 'react'
import { Component } from 'react'

import { Renderer } from '../../../render2d/renderer'
import { ChartModel } from '../model'

import { LineChartController } from './controller'
import { LineChartModel } from './model'
import style from './style.css'
import { LineChartViewModel } from './view_model'

export type LineChartProps = {}

@observer
export class LineChart extends Component<
  LineChartProps & {
    model: LineChartModel
    controller: LineChartController
  }
> {
  static of(model: ChartModel): ComponentType<LineChartProps> {
    const controller = LineChartController.of()
    const lineChartModel = LineChartModel.of(model)
    return props => <LineChart {...props} model={lineChartModel} controller={controller} />
  }

  render() {
    const viewModel = LineChartViewModel.of(this.props.model)
    const { bufferSeconds, minValue, maxValue } = viewModel

    return (
      <>
        <div className={style.topBar}>
          <label className={style.topBarItem}>
            Minimum Value
            <input
              className={style.topBarInput}
              type="number"
              onChange={this.onChangeMin}
              placeholder={`(${minValue.toPrecision(3)})`}
            />
          </label>
          <label className={style.topBarItem}>
            Maximum Value
            <input
              className={style.topBarInput}
              type="number"
              onChange={this.onChangeMax}
              placeholder={`(${maxValue.toPrecision(3)})`}
            />
          </label>
          <label className={style.topBarItem}>
            View Seconds
            <input
              className={style.topBarInput}
              type="number"
              onChange={this.onChangeBuffer}
              placeholder={`(${bufferSeconds.toPrecision(3)})`}
            />
          </label>
        </div>
        <div className={style.container}>
          <Renderer
            engine="svg"
            className={style.field}
            scene={viewModel.scene}
            camera={viewModel.camera}
          />
        </div>
      </>
    )
  }

  private readonly onChangeMin = (event: ChangeEvent<HTMLInputElement>) => {
    const { controller, model } = this.props
    controller.onChangeMin(model, event.target.value)
  }

  private readonly onChangeMax = (event: ChangeEvent<HTMLInputElement>) => {
    const { controller, model } = this.props
    controller.onChangeMax(model, event.target.value)
  }

  private readonly onChangeBuffer = (event: ChangeEvent<HTMLInputElement>) => {
    const { controller, model } = this.props
    controller.onChangeBuffer(model, event.target.value)
  }
}
