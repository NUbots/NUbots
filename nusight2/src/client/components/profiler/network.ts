import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { Profile, ProfilerRobotModel } from "./model";
import ReactionProfiles = message.support.nuclear.ReactionProfiles;

export class ProfilerNetwork {
  constructor(private network: Network) {
    this.network.on(ReactionProfiles, this.onReactionProfiles);
  }

  static of(nusightNetwork: NUsightNetwork): ProfilerNetwork {
    const network = Network.of(nusightNetwork);
    return new ProfilerNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onReactionProfiles(robotModel: RobotModel, profiles: ReactionProfiles) {
    const robot = ProfilerRobotModel.of(robotModel);

    // Clear existing profiles
    robot.profiles = [];

    // Update with new profiles
    profiles.reactionProfiles!.forEach((p) => {
      const newProfile = new Profile(
        p.name!,
        p.reactionId!,
        p.reactor!,
        p.totalTime!,
        p.count!,
        p.minTime!,
        p.maxTime!,
        p.avgTime!,
        p.percentage!,
      );
      robot.profiles.push(newProfile);
    });
  }
}
