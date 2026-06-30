import { DirectorState } from "@proto/message/behaviour/Director";
import { action } from "mobx";

import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { DirectorModel, transformDirectorState } from "./model";

export class DirectorNetwork {
  constructor(
    private model: DirectorModel,
    private network: Network,
  ) {
    this.network.on(DirectorState, this.onDirectorState);
  }

  static of(nusightNetwork: NUsightNetwork, model: DirectorModel): DirectorNetwork {
    const network = Network.of(nusightNetwork);
    return new DirectorNetwork(model, network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onDirectorState(robotModel: RobotModel, state: DirectorState) {
    const graph = transformDirectorState(state);
    this.model.graphsByRobot.set(robotModel.id, graph);
  }
}
