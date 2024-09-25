import bounds from "binary-search-bounds";
import { action } from "mobx";

import { BrowserSystemClock } from "../../../client/time/browser_clock";
import { Vector2 } from "../../../shared/math/vector2";
import { message } from "../../../shared/messages";
import { Clock } from "../../../shared/time/clock";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { RerunModel } from "./model";

export class RerunNetwork {
  constructor(private clock: Clock, private network: Network, private model: RerunModel) {}

  static of(nusightNetwork: NUsightNetwork, model: RerunModel): RerunNetwork {
    const network = Network.of(nusightNetwork);
    return new RerunNetwork(BrowserSystemClock, network, model);
  }

  destroy() {
    this.network.off();
  }
}
