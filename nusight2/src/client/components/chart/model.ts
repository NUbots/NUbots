import { observable } from "mobx";
import { computed } from "mobx";

import { Vector2 } from "../../../shared/math/vector2";
import { BrowserSystemClock } from "../../time/browser_clock";
import { CheckedState } from "../checkbox_tree/model";
import { RobotModel } from "../robot/model";

import { TreeViewModel } from "./view_model";

export interface TreeData extends Map<string, TreeData | DataSeries> {}

export class DataSeries {
  private kf: { processNoise: number; measurementNoise: number };

  @observable accessor highlight: boolean = false;
  @observable accessor color: string;
  @observable accessor startTime: number;
  @observable accessor timeDelta: number;
  @observable accessor timeVariance: number;
  @observable accessor checked: CheckedState;
  @observable accessor series: Vector2[];

  constructor({
    color,
    checked,
    series,
    startTime,
    timeDelta,
    timeVariance,
    kf,
  }: {
    color: string;
    checked: CheckedState;
    series: Vector2[];
    startTime: number;
    timeDelta: number;
    timeVariance: number;
    kf: { processNoise: number; measurementNoise: number };
  }) {
    this.color = color;
    this.checked = checked;
    this.series = series;
    this.startTime = startTime;
    this.timeDelta = timeDelta;
    this.timeVariance = timeVariance;
    this.kf = kf;
  }

  static of(startTime: number = 0) {
    return new DataSeries({
      color: "#ffffff",
      checked: CheckedState.Unchecked,
      series: [],
      startTime,
      timeDelta: 0,
      timeVariance: 1,
      kf: { processNoise: 1e-3, measurementNoise: 1e-1 },
    });
  }

  updateDelta(delta: number) {
    const Q = this.kf.processNoise;
    const R = this.kf.measurementNoise;

    // Calculate kalman gain
    const K = (this.timeVariance + Q) / (this.timeVariance + Q + R);

    // Do the filter
    this.timeVariance = (R * (this.timeVariance + Q)) / (R + this.timeVariance + Q);
    this.timeDelta = this.timeDelta + (delta - this.timeDelta) * K;
  }
}

type ChartModelOpts = {
  robotModels: RobotModel[];
};

export class ChartModel {
  @observable private accessor robotModels: RobotModel[];

  @observable accessor treeData: TreeData;
  @observable accessor startTime: number;
  @observable accessor bufferSeconds: number;

  constructor({
    robotModels,
    treeData,
    startTime,
    bufferSeconds,
  }: {
    robotModels: RobotModel[];
    treeData: Map<string, TreeData | DataSeries>;
    startTime: number;
    bufferSeconds: number;
  }) {
    this.robotModels = robotModels;
    this.treeData = treeData;
    this.startTime = startTime;
    this.bufferSeconds = bufferSeconds;
  }

  static of({ robotModels }: ChartModelOpts) {
    return new ChartModel({
      robotModels,
      treeData: new Map(),
      // The exact value of this number isn't important, it's simply here to make the time numbers smaller
      startTime: BrowserSystemClock.now(),
      bufferSeconds: 10,
    });
  }

  @computed
  get tree() {
    return {
      nodes: Array.from(this.treeData.entries(), ([label, model]) => TreeViewModel.of({ label, model })),
      usePessimisticToggle: true,
    };
  }
}
