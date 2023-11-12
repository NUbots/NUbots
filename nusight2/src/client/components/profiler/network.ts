import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { ProfilerRobotModel } from "./model";
import { Profile } from "./model";

import ReactionProfile = message.support.nuclear.ReactionProfile;

export class ProfilerNetwork {
  constructor(private network: Network) {
    this.network.on(ReactionProfile, this.onReactionProfile);
  }

  static of(nusightNetwork: NUsightNetwork): ProfilerNetwork {
    const network = Network.of(nusightNetwork);
    return new ProfilerNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onReactionProfile(robotModel: RobotModel, profileData: ReactionProfile) {
    console.log("onReactionProfile");

    const robot = ProfilerRobotModel.of(robotModel);

    // Find the index of the existing profile with the same name
    const profileIndex = robot.profiles.findIndex(p => p.name === profileData.name);

    if (profileIndex !== -1) {
      // Profile exists, update it
      robot.profiles[profileIndex] = new Profile(
        profileData.name,
        profileData.totalTime,
        profileData.count,
        profileData.minTime,
        profileData.maxTime,
        profileData.avgTime
      );
    } else {
      // Profile does not exist, add it
      robot.profiles.push(new Profile(
        profileData.name,
        profileData.totalTime,
        profileData.count,
        profileData.minTime,
        profileData.maxTime,
        profileData.avgTime
      ));
    }
  }
}
