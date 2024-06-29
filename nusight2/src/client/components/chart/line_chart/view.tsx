import { ChangeEvent } from "react";
import { ComponentType } from "react";
import React from "react";
import { Component } from "react";
import { observer } from "mobx-react";

import { Renderer } from "../../../render2d/renderer";
import { ChartModel } from "../model";

import { LineChartController } from "./controller";
import { LineChartModel } from "./model";
import { LineChartViewModel } from "./view_model";

export type LineChartProps = {};

@observer
export class LineChart extends Component<
  LineChartProps & {
    model: LineChartModel;
    controller: LineChartController;
  }
> {
  static of(model: ChartModel): ComponentType<LineChartProps> {
    const controller = LineChartController.of();
    const lineChartModel = LineChartModel.of(model);
    return (props) => <LineChart {...props} model={lineChartModel} controller={controller} />;
  }

  render() {
    const viewModel = LineChartViewModel.of(this.props.model);
    const { bufferSeconds, minValue, maxValue } = viewModel;

    return (
      <>
        <div className="bg-auto-surface-2 border-y border-auto py-2 px-3">
          <label className="pr-4 shrink-0">
            Minimum Value
            <input
              className="bg-auto-surface-1 placeholder-auto-secondary shadow-inner focus:outline-none focus:outline-blue-500 px-2 py-1 rounded-sm ml-2 w-24"
              type="number"
              onChange={this.onChangeMin}
              placeholder={`(${minValue.toPrecision(3)})`}
            />
          </label>
          <label className="pr-4 shrink-0">
            Maximum Value
            <input
              className="bg-auto-surface-1 placeholder-auto-secondary shadow-inner focus:outline-none focus:outline-auto-divider px-2 py-1 rounded-sm ml-2 w-24"
              type="number"
              onChange={this.onChangeMax}
              placeholder={`(${maxValue.toPrecision(3)})`}
            />
          </label>
          <label className="pr-4 shrink-0">
            View Seconds
            <input
              className="bg-auto-surface-1 placeholder-auto-secondary shadow-inner focus:outline-none focus:outline-auto-divider px-2 py-1 rounded-sm ml-2 w-24"
              type="number"
              onChange={this.onChangeBuffer}
              placeholder={`(${bufferSeconds.toPrecision(3)})`}
            />
          </label>
        </div>
        <div className="flex-grow relative">
          <Renderer engine="svg" scene={viewModel.scene} camera={viewModel.camera} />
        </div>
      </>
    );
  }

  private readonly onChangeMin = (event: ChangeEvent<HTMLInputElement>) => {
    const { controller, model } = this.props;
    controller.onChangeMin(model, event.target.value);
  };

  private readonly onChangeMax = (event: ChangeEvent<HTMLInputElement>) => {
    const { controller, model } = this.props;
    controller.onChangeMax(model, event.target.value);
  };

  private readonly onChangeBuffer = (event: ChangeEvent<HTMLInputElement>) => {
    const { controller, model } = this.props;
    controller.onChangeBuffer(model, event.target.value);
  };
}
