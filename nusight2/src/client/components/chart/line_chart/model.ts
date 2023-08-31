import { computed } from "mobx";
import { observable } from "mobx";

import { ChartModel } from "../model";

export type LineChartModelOpts = {
  model: ChartModel;
};

export class LineChartModel {
  @observable model: ChartModel;
  @observable yMin: number | "auto";
  @observable yMax: number | "auto";

  constructor(opts: LineChartModelOpts) {
    this.model = opts.model;
    this.yMin = "auto";
    this.yMax = "auto";
  }

  static of(model: ChartModel): LineChartModel {
    return new LineChartModel({ model });
  }

  @computed
  get startTime() {
    return this.model.startTime;
  }

  @computed
  get treeData() {
    return this.model.treeData;
  }

  @computed
  get bufferSeconds() {
    return this.model.bufferSeconds;
  }

  set bufferSeconds(v: number) {
    this.model.bufferSeconds = v;
  }
}
