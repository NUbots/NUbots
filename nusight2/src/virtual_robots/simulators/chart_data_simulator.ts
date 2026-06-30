import { DataPoint } from "@proto/message/eye/DataPoint";
import { autorun } from "mobx";

import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { Simulator } from "../simulator";
import { Message } from "../simulator";

import { periodic } from "./periodic";

export class ChartSimulator extends Simulator {
  static of({ nuclearnetClient }: { nuclearnetClient: NUClearNetClient }): ChartSimulator {
    return new ChartSimulator(nuclearnetClient);
  }

  start() {
    return autorun(() => this.send(this.chartData));
  }

  get chartData(): Message {
    // Offset our time to test the adaptive window
    const time = periodic(60) - 3;

    const messageType = "message.eye.DataPoint";
    const period = 10;
    const theta = (2 * Math.PI * time) / period;
    const sin = Math.sin(theta);
    const cos = Math.cos(theta);

    const buffer = new DataPoint({
      label: "Debug Waves",
      value: [sin, cos, 2 * sin, 4 * cos],
      timestamp: { seconds: BigInt(Math.floor(time)), nanos: Math.floor((time * 1e9) % 1e9) },
    }).toBinary();

    const message = { messageType, buffer };
    return message;
  }
}
