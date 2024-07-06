import { action } from "mobx";

import { message } from "../../../shared/messages";
import { NbsScrubber } from "../../../shared/nbs_scrubber";
import { NUClearNetPeerWithType } from "../../../shared/nuclearnet/nuclearnet_client";
import { TimestampObject } from "../../../shared/time/timestamp";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { NbsScrubberModel } from "../nbs_scrubbers/model";
import { RobotModel } from "../robot/model";

import { AppModel } from "./model";

import ScrubberState = message.eye.ScrubberState;

export class AppNetwork {
  private nextRobotId: number;

  constructor(private nusightNetwork: NUsightNetwork, private model: AppModel) {
    this.nextRobotId = 0;

    nusightNetwork.onNUClearJoin(this.onJoin);
    nusightNetwork.onNUClearLeave(this.onLeave);

    const network = Network.of(nusightNetwork);
    network.on(message.eye.ScrubberState, this.onScrubberState);
    network.on(message.eye.ScrubberClosed, this.onScrubberClosed);
  }

  static of(nusightNetwork: NUsightNetwork, model: AppModel) {
    return new AppNetwork(nusightNetwork, model);
  }

  @action
  private onJoin = (peer: NUClearNetPeerWithType) => {
    if (peer.name === "nusight") {
      return;
    }

    const robot = this.findRobot(peer);

    if (robot) {
      robot.connected = true;
      // Keep this in sync, since the port will likely change per connection.
      robot.port = peer.port;
    } else {
      const robotId = String(this.nextRobotId++);
      this.model.robots.push(
        RobotModel.of({
          id: robotId,
          name: peer.name,
          address: peer.address,
          port: peer.port,
          connected: true,
          enabled: true,
          type: peer.type ?? "nuclearnet-peer",
        }),
      );
    }
  };

  @action
  private onLeave = (peer: NUClearNetPeerWithType) => {
    if (peer.name === "nusight") {
      return;
    }

    const robot = this.model.robots.find((otherRobot) => {
      return otherRobot.name === peer.name && otherRobot.address === peer.address && otherRobot.port === peer.port;
    });
    if (robot) {
      robot.connected = false;
    }
  };

  private findRobot(peer: NUClearNetPeerWithType): RobotModel | undefined {
    const candidates = this.model.robots.filter((otherRobot) => {
      return otherRobot.name === peer.name && otherRobot.address === peer.address;
    });
    /*
     * Robots are not always uniquely identifiable across connections since they may have the same name and address.
     * We essentially guess and pick the first candidate.
     */
    return candidates.length > 0 ? candidates[0] : undefined;
  }

  @action
  private onScrubberState = (robot: RobotModel, message: message.eye.ScrubberState) => {
    const scrubber = this.model.scrubbersModel.scrubbers.get(message.id);

    if (scrubber) {
      scrubber.startTs.seconds = message.start!.seconds!;
      scrubber.startTs.nanos = message.start!.nanos!;

      scrubber.endTs.seconds = message.end!.seconds!;
      scrubber.endTs.nanos = message.end!.nanos!;

      scrubber.playbackRepeat = message.playbackRepeat;
      scrubber.playbackSpeed = message.playbackSpeed;
      scrubber.playbackState = scrubberPlaybackStateFromEnum[message.playbackState];

      // If we're not currently seeking in the UI, update the current timestamp
      if (!scrubber.isSeeking) {
        scrubber.current = TimestampObject.toNanos(message.timestamp);
      }
    } else {
      const scrubber = new NbsScrubberModel({
        id: message.id,
        name: message.name,
        start: {
          seconds: message.start!.seconds!,
          nanos: message.start!.nanos!,
        },
        end: {
          seconds: message.end!.seconds!,
          nanos: message.end!.nanos!,
        },
        playbackRepeat: message.playbackRepeat,
        playbackSpeed: message.playbackSpeed,
        playbackState: scrubberPlaybackStateFromEnum[message.playbackState],
      });
      scrubber.current = TimestampObject.toNanos(message.timestamp);

      this.model.scrubbersModel.scrubbers.set(scrubber.id, scrubber);
    }
  };

  @action
  private onScrubberClosed = (robotModel: RobotModel, msg: message.eye.ScrubberClosed) => {
    this.model.scrubbersModel.scrubbers.delete(msg.id);
  };
}

const scrubberPlaybackStateFromEnum: Record<ScrubberState.State, NbsScrubber["playbackState"]> = {
  [ScrubberState.State.PAUSED]: "paused",
  [ScrubberState.State.PLAYING]: "playing",
  [ScrubberState.State.ENDED]: "ended",
  [ScrubberState.State.UNKNOWN]: "unknown",
} as const;
