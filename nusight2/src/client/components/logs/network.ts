import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { LogsRobotModel } from "./model";

import LogMessage = message.nuclear.LogMessage;

export class LogsNetwork {
  constructor(private network: Network) {
    this.network.on(LogMessage, this.onLogMessage);
  }

  static of(nusightNetwork: NUsightNetwork): LogsNetwork {
    const network = Network.of(nusightNetwork);
    return new LogsNetwork(network);
  }

  destroy() {
    this.network.off();
  }

  @action.bound
  private onLogMessage(robotModel: RobotModel, message: LogMessage) {
    const robot = LogsRobotModel.of(robotModel);

    robot.messages.push({
      message: message.message,
    });
  }
}
