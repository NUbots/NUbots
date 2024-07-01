import { action } from "mobx";

import { google, message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { DirectorRobotModel } from "./model";

import DirectorMessage = message.behaviour.Director;

export class DirectorNetwork {
  constructor(private network: Network) {
    this.network.on(DirectorMessage, this.onDirectorMessage);
  }

  static of(nusightNetwork: NUsightNetwork): DirectorNetwork {
    const network = Network.of(nusightNetwork);
    return new DirectorNetwork(network);
  }

  destroy() {
    this.network.off();
  }

  @action.bound
  private onDirectorMessage(robotModel: RobotModel, message: DirectorMessage) {
    const robot = DirectorRobotModel.of(robotModel);
    // console.log("Received director message: ", message);
    // console.log("Robot: ", robot);
    // Set a robot parameter from the message
    robot.providers = [];
    message.providers.forEach((provider) => {
      const newProvider = {
        name: provider.name ?? "",
        active: provider.active ?? false,
        done: provider.done ?? false,
      };
      robot.providers.push(newProvider);
    });
    // console.log(robot.providers)
    // for (const provider of message.providers) {
    //   robot.providers.push({
    //     name: provider.name ?? "",
    //     active: provider.active ?? false,
    //     done: provider.done ?? false,
    //   });
    // console.log(robot.providers[1])
    // }
  }
}
