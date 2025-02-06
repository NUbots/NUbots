import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";

export class KinematicsNetwork {
  constructor(private network: Network) {
    this.network.on(message.input.Sensors, this.onMessageReceived);
  }

  static of(nusightNetwork: NUsightNetwork): KinematicsNetwork {
    const network = Network.of(nusightNetwork);
    return new KinematicsNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onMessageReceived(data: any) {
    console.log(data);
  }
}
