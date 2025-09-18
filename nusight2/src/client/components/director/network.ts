import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { DirectorModel, transformDirectorState } from "./model";

export class DirectorNetwork {
  constructor(
    private model: DirectorModel,
    private network: Network,
  ) {
    this.network.on(message.behaviour.DirectorState, this.onDirectorState);
  }

  static of(nusightNetwork: NUsightNetwork, model: DirectorModel): DirectorNetwork {
    const network = Network.of(nusightNetwork);
    return new DirectorNetwork(model, network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onDirectorState(_robot: RobotModel, state: message.behaviour.DirectorState) {
    this.model.graph = transformDirectorState(state);
  }
}
