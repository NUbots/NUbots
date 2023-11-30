import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";

export class LogsNetwork {
  constructor(private network: Network) {}

  static of(nusightNetwork: NUsightNetwork): LogsNetwork {
    const network = Network.of(nusightNetwork);
    return new LogsNetwork(network);
  }

  destroy() {
    this.network.off();
  }
}
