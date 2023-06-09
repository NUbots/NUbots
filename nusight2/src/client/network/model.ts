import { observable } from "mobx";

import { memoize } from "../base/memoize";
import { RobotModel } from "../components/robot/model";
import { BrowserSystemClock } from "../time/browser_clock";

import { Rate } from "./rate";

export class RobotNetworkStatsModel {
  @observable.ref packets: number;
  @observable.ref packetsPerSecond: Rate;
  @observable.ref bytes: number;
  @observable.ref bytesPerSecond: Rate;

  constructor({
    packets,
    packetsPerSecond,
    bytes,
    bytesPerSecond,
  }: {
    packets: number;
    packetsPerSecond: Rate;
    bytes: number;
    bytesPerSecond: Rate;
  }) {
    this.packets = packets;
    this.packetsPerSecond = packetsPerSecond;
    this.bytes = bytes;
    this.bytesPerSecond = bytesPerSecond;
  }

  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  static of = memoize((robotModel: RobotModel) => {
    return new RobotNetworkStatsModel({
      bytes: 0,
      bytesPerSecond: new Rate({ smoothing: 0.9 }, BrowserSystemClock),
      packets: 0,
      packetsPerSecond: new Rate({ smoothing: 0.9 }, BrowserSystemClock),
    });
  });
}
