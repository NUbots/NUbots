import { action } from "mobx";

import { message } from "../../../shared/messages";
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

    robot.layers.clear();
    robot.children.clear();
    message.providers.forEach((provider) => {
      let newProvider: {
        id: string;
        layer: string;
        name: string;
        parent: string;
        active: boolean;
        done: boolean;
        isRootTask: boolean;
      };

      if (provider.name?.startsWith("extension::behaviour::commands::RootType")) {
        newProvider = {
          id: provider.name ?? "",
          layer: "root",
          name: provider.name?.match(/^extension::behaviour::commands::RootType<message::(.*)>/)?.[1] ?? "",
          parent: "",
          active: provider.active ?? false,
          done: provider.done ?? false,
          isRootTask: true,
        };
      } else {
        newProvider = {
          id: provider.name ?? "",
          layer: provider.name?.match(/^message::(.*?)::(.*)/)?.[1] ?? "",
          name: provider.name?.match(/^message::(.*?)::(.*)/)?.[2] ?? "",
          parent: provider.parent ?? "",
          active: provider.active ?? false,
          done: provider.done ?? false,
          isRootTask: provider.name?.match(/^message::(.*?)::(.*)/)?.[1] === "root",
        };
      }

      if (provider.parent) {
        if (!robot.children.has(provider.parent)) {
          robot.children.set(provider.parent, []);
        }
        robot.children.get(provider.parent)?.push(newProvider);
      }

      if (!robot.layers.has(newProvider.layer)) {
        robot.layers.set(newProvider.layer, []);
      }
      robot.layers.get(newProvider.layer)?.push(newProvider);
    });
  }
}
