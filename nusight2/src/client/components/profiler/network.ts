import { action } from "mobx";

import { message } from "../../../shared/messages";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { ProfilerRobotModel, Profile } from "./model";
import ReactionStatistics = message.nuclear.ReactionStatistics;

export class ProfilerNetwork {
  constructor(private network: Network) {
    this.network.on(ReactionStatistics, this.onReactionStatistics);
  }

  static of(nusightNetwork: NUsightNetwork): ProfilerNetwork {
    const network = Network.of(nusightNetwork);
    return new ProfilerNetwork(network);
  }

  destroy = () => {
    this.network.off();
  };

  @action.bound
  private onReactionStatistics(robotModel: RobotModel, stats: ReactionStatistics) {
    console.log("onReactionStatistics");

    const robot = ProfilerRobotModel.of(robotModel);

    // Compute the time of the reaction
    const startTime = stats.started!.seconds * 1000 + stats.started!.nanos! / 1000000;
    const finishTime = stats.finished!.seconds * 1000 + stats.finished!.nanos! / 1000000;
    const time = finishTime - startTime; // Time in milliseconds

    // Find the index of the existing profile with the same reactionId
    const profileIndex = robot.profiles.findIndex(p => p.reactionId === stats.reactionId);

    if (profileIndex !== -1) {
      // Profile exists, update it
      robot.profiles[profileIndex].updateProfile(time, robot.profiles[profileIndex].total_time);
    } else {
      // Profile does not exist, create and add it
      const newProfile = new Profile(
        stats.identifiers!.name!,
        stats.reactionId,
        stats.identifiers!.reactor!,
        time,
        1,
        time,
        time,
        time,
        0 // Initial percentage, will be recalculated
      );
      robot.profiles.push(newProfile);
    }

    // Recalculate percentages for all profiles
    let total_time_all = robot.profiles.reduce((acc, p) => acc + p.total_time, 0);
    robot.profiles.forEach(p => {
      p.percentage = 100.0 * p.total_time / total_time_all;
    });
  }
}
