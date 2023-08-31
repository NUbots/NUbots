import { action } from "mobx";
import { NUClearNetPacket } from "nuclearnet.js";

import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { Simulator } from "../simulator";

export class ScriptDataSimulator extends Simulator {
  constructor(nuclearnetClient: NUClearNetClient) {
    super(nuclearnetClient);
    this.nuclearnetClient.on("message.input.Sensors", this.onSensors);
  }

  static of({ nuclearnetClient }: { nuclearnetClient: NUClearNetClient }) {
    return new ScriptDataSimulator(nuclearnetClient);
  }

  start() {
    // TODO
    return () => 0;
  }

  @action.bound
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  onSensors(packet: NUClearNetPacket) {
    // console.log('ScriptDataSimulator: packet received', packet)
  }
}
