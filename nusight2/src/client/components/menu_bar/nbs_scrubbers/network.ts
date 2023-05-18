import { action } from "mobx";

import { google, message } from "../../../../shared/messages";
import { RpcResult } from "../../../../shared/messages/rpc_call";
import { NbsScrubber } from "../../../../shared/nbs_scrubber";
import { Network } from "../../../network/network";
import { NUsightNetwork } from "../../../network/nusight_network";
import { RobotModel } from "../../robot/model";

import { NbsScrubbersModel } from "./model";
import { NbsScrubberModel } from "./nbs_scrubber/model";
import { timestampObjectToNanos, timestampToPercentage } from "./util";

import ScrubberState = message.eye.ScrubberState;

export class NbsScrubbersNetwork {
  constructor(private network: Network, private model: NbsScrubbersModel) {
    this.network.on(message.eye.ScrubberState, this.onScrubberState);
    this.network.on(message.eye.ScrubberClosed, this.onScrubberClosed);
  }

  static of(nusightNetwork: NUsightNetwork, model: NbsScrubbersModel): NbsScrubbersNetwork {
    const network = Network.of(nusightNetwork);
    return new NbsScrubbersNetwork(network, model);
  }

  destroy() {
    this.network.off();
  }

  async listScrubberFiles(request: message.eye.IScrubberListFilesRequest) {
    const result = await this.network.call(new message.eye.ScrubberListFilesRequest(request));

    if (result.ok) {
      this.onScrubberListFilesSuccess(result.data.response);
    } else {
      if (result.error.isRemoteError()) {
        this.onScrubberListFilesError(result.error.message);
      } else {
        result.error.defaultHandler();
      }
    }
  }

  async loadScrubber(request: message.eye.IScrubberLoadRequest) {
    const result = await this.network.call(new message.eye.ScrubberLoadRequest(request));

    if (!result.ok) {
      if (result.error.isRemoteError() || result.error.isTimeout()) {
        this.onScrubberError(result.error.message, "Scrubber load error");
      } else {
        result.error.defaultHandler();
      }
    } else {
      // The new scrubber will be added when we get its initial state in onScrubberState()
    }
  }

  async playScrubber(request: message.eye.IScrubberPlayRequest) {
    const result = await this.network.call(new message.eye.ScrubberPlayRequest(request));
    this.defaultRpcResultHandler(result);
  }

  async pauseScrubber(request: message.eye.IScrubberPauseRequest) {
    const result = await this.network.call(new message.eye.ScrubberPauseRequest(request));
    this.defaultRpcResultHandler(result);
  }

  async seekScrubber(request: message.eye.IScrubberSeekRequest) {
    const result = await this.network.call(new message.eye.ScrubberSeekRequest(request));
    this.defaultRpcResultHandler(result);
  }

  async setScrubberPlaybackSpeed(request: message.eye.IScrubberSetPlaybackSpeedRequest) {
    const result = await this.network.call(new message.eye.ScrubberSetPlaybackSpeedRequest(request));
    this.defaultRpcResultHandler(result);
  }

  async setScrubberRepeat(request: message.eye.IScrubberSetRepeatRequest) {
    const result = await this.network.call(new message.eye.ScrubberSetRepeatRequest(request));
    this.defaultRpcResultHandler(result);
  }

  async closeScrubber(request: message.eye.IScrubberCloseRequest) {
    const result = await this.network.call<message.eye.ScrubberCloseRequest.Response>(
      new message.eye.ScrubberCloseRequest(request),
    );

    if (!result.ok) {
      if (result.error.isRemoteError() || result.error.isTimeout()) {
        this.onScrubberError(result.error.message, "Scrubber close error");
      } else {
        result.error.defaultHandler();
      }
    } else {
      // The scrubber will be removed when we get the closed message in onScrubberClosed()
    }
  }

  @action
  private onScrubberListFilesSuccess = (response: message.eye.ScrubberListFilesRequest.Response) => {
    this.model.filePicker.error = "";
    this.model.filePicker.currentPath = response.directory!;
    this.model.filePicker.currentPathEntries = (response.entries ?? []).map((entry) => {
      const seconds = entry.dateModified!.seconds + entry.dateModified!.nanos! / 1e9;
      return {
        type: entry.type === message.eye.ScrubberFileEntry.Type.DIRECTORY ? "directory" : "file",
        name: entry.name!,
        path: entry.path!,
        size: entry.size!,
        dateModified: new Date(seconds * 1000),
      };
    });
  };

  @action
  private onScrubberListFilesError = (error: string) => {
    this.model.filePicker.error = error;
  };

  @action
  private onScrubberState = (robot: RobotModel, message: message.eye.ScrubberState) => {
    const data = {
      id: message.id,
      name: message.name,
      start: {
        seconds: Number(message.start?.seconds!),
        nanos: Number(message.start?.nanos!),
      },
      end: {
        seconds: Number(message.end?.seconds!),
        nanos: Number(message.end?.nanos!),
      },
      playbackRepeat: message.playbackRepeat,
      playbackSpeed: message.playbackSpeed,
      playbackState: scrubberPlaybackStateFromEnum[message.playbackState],
    };

    const scrubber = this.model.scrubbers.get(message.id);

    if (scrubber) {
      scrubber.id = data.id;
      scrubber.name = data.name;
      scrubber.startTs = data.start;
      scrubber.endTs = data.end;
      scrubber.playbackRepeat = data.playbackRepeat;
      scrubber.playbackSpeed = data.playbackSpeed;
      scrubber.playbackState = data.playbackState;

      if (!scrubber.isSeeking) {
        scrubber.percentPlayed = protobufTimestampToPercentage(message.timestamp!, scrubber.start, scrubber.end);
      }
    } else {
      const scrubber = new NbsScrubberModel(data);
      scrubber.percentPlayed = protobufTimestampToPercentage(message.timestamp!, scrubber.start, scrubber.end);

      this.model.scrubbers.set(scrubber.id, scrubber);

      // Show the scrubbers, to reveal the new scrubber
      this.model.showScrubbers = true;
    }
  };

  @action
  private onScrubberClosed = (robotModel: RobotModel, response: message.eye.ScrubberClosed) => {
    this.model.scrubbers.delete(response.id);

    if (this.model.scrubbers.size === 0) {
      this.model.showScrubbers = false;
    }
  };

  @action
  private defaultRpcResultHandler = <T>(result: RpcResult<T>) => {
    if (!result.ok) {
      if (result.error.isRemoteError()) {
        this.onScrubberError(result.error.message);
      } else {
        result.error.defaultHandler();
      }
    }
  };

  @action
  private onScrubberError = (error: string, prefix?: string) => {
    // TODO: add toast messages to NUsight to show errors like this without using alert()
    alert(`${prefix ? prefix + ":\n" : ""}${error}`);
  };
}

const scrubberPlaybackStateFromEnum: Record<ScrubberState.State, NbsScrubber["playbackState"]> = {
  [ScrubberState.State.PAUSED]: "paused",
  [ScrubberState.State.PLAYING]: "playing",
  [ScrubberState.State.ENDED]: "ended",
  [ScrubberState.State.UNKNOWN]: "unknown",
} as const;

function protobufTimestampToPercentage(timestamp: google.protobuf.ITimestamp, startNanos: bigint, endNanos: bigint) {
  const timestampNanos = timestampObjectToNanos({
    seconds: Number(timestamp?.seconds!),
    nanos: Number(timestamp?.nanos!),
  });

  return timestampToPercentage(timestampNanos, startNanos, endNanos);
}
