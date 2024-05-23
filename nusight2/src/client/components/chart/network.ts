import bounds from "binary-search-bounds";
import { action } from "mobx";

import { BrowserSystemClock } from "../../../client/time/browser_clock";
import { Vector2 } from "../../../shared/math/vector2";
import { message } from "../../../shared/messages";
import { Clock } from "../../../shared/time/clock";
import { TimestampObject } from "../../../shared/time/timestamp";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { ChartModel } from "./model";
import { DataSeries } from "./model";
import { TreeData } from "./model";
import DataPoint = message.eye.DataPoint;

export class ChartNetwork {
  constructor(private clock: Clock, private network: Network, private model: ChartModel) {
    this.network.on(DataPoint, this.onDataPoint);
  }

  static of(nusightNetwork: NUsightNetwork, model: ChartModel): ChartNetwork {
    const network = Network.of(nusightNetwork);
    return new ChartNetwork(BrowserSystemClock, network, model);
  }

  destroy() {
    this.network.off();
  }

  @action
  private onDataPoint = (robotModel: RobotModel, data: DataPoint) => {
    if (data.value.length === 0) {
      return;
    }

    const basePath = [robotModel.name].concat(data.label.split("/"));
    const keys =
      data.value.length === 1
        ? [basePath.pop()]
        : data.value.length < 5
        ? ["x", "y", "z", "w"]
        : data.value.map((v, i) => `s${i}`);

    const node = basePath.reduce((accumulator: TreeData, p: string) => {
      if (!accumulator.has(p)) {
        accumulator.set(p, new Map<string, TreeData | DataSeries>());
      }
      return accumulator.get(p)! as TreeData;
    }, this.model.treeData);

    data.value.forEach((v, i) => {
      const key = keys[i]!;

      if (!node.has(key)) {
        // Create a new series with the start time of this datapoint
        node.set(key, DataSeries.of(TimestampObject.toSeconds(data.timestamp)));
      }

      const leaf = node.get(key) as DataSeries;
      const series = leaf.series;

      // Now according to the chart timespace
      const chartTime = this.clock.now() - this.model.startTime;

      // Now according to the datapoint
      const pointTime = TimestampObject.toSeconds(data.timestamp) - leaf.startTime;

      // Estimate the drifting distance between the clocks
      leaf.updateDelta(pointTime - chartTime);

      // Add the series element
      series.push(Vector2.of(pointTime, v));

      // Swap it backward until it's in place (keeping the list sorted)
      for (let i = series.length - 1; i > 0; i--) {
        if (series[i - 1].x > pointTime) {
          [series[i - 1], series[i]] = [series[i], series[i - 1]];
        } else {
          break;
        }
      }

      // Find where our old data starts so we can remove it
      const cutoff = chartTime + leaf.timeDelta - this.model.bufferSeconds;
      const newStart = bounds.lt(series, Vector2.of(), (p) => p.x - cutoff);

      // Remove old series elements in batches
      if (newStart > 50) {
        series.splice(0, newStart);
      }
    });
  };
}
