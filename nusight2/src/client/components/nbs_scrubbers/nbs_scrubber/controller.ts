import {
  ScrubberCloseRequest,
  ScrubberPauseRequest,
  ScrubberPlayRequest,
  ScrubberSeekRequest,
  ScrubberSeekRequest_Response,
  ScrubberSetPlaybackSpeedRequest,
  ScrubberSetRepeatRequest,
} from "@proto/message/eye/Scrubber";
import { action } from "mobx";

import { RpcResult } from "../../../../shared/messages/generated/rpc_call";
import { Timestamp } from "../../../../shared/time/timestamp";
import { RpcNetwork } from "../../../hooks/use_rpc_controller";
import { NbsScrubberModel } from "../model";
import { percentageToTimestamp } from "../util";

export class NbsScrubberController {
  constructor(
    private scrubber: NbsScrubberModel,
    private network: RpcNetwork,
  ) {}

  close = () => {
    this.network.call(new ScrubberCloseRequest({ id: this.scrubber.id })).then((result) => {
      if (!result.ok) {
        if (result.error.isRemoteError() || result.error.isTimeout()) {
          this.onScrubberError(result.error.message, "Scrubber close error");
        } else {
          result.error.defaultHandler();
        }
      } else {
        // The scrubber will be removed when we get the closed message in onScrubberClosed()
      }
    });
  };

  togglePlayback = () => {
    if (this.scrubber.playbackState === "playing") {
      this.network.call(new ScrubberPauseRequest({ id: this.scrubber.id })).then(this.defaultRpcResultHandler);
    } else {
      this.network.call(new ScrubberPlayRequest({ id: this.scrubber.id })).then(this.defaultRpcResultHandler);
    }
  };

  playFaster = () => {
    const request = new ScrubberSetPlaybackSpeedRequest({
      id: this.scrubber.id,
      playbackSpeed: this.scrubber.playbackSpeed + 1,
    });
    this.network.call(request).then(this.defaultRpcResultHandler);
  };

  playSlower = () => {
    const request = new ScrubberSetPlaybackSpeedRequest({
      id: this.scrubber.id,
      playbackSpeed: this.scrubber.playbackSpeed - 1,
    });
    this.network.call(request).then(this.defaultRpcResultHandler);
  };

  toggleRepeat = () => {
    const request = new ScrubberSetRepeatRequest({
      id: this.scrubber.id,
      repeat: !this.scrubber.playbackRepeat,
    });
    this.network.call(request).then(this.defaultRpcResultHandler);
  };

  seekToPercent = (percentPlayed: number) => {
    this.seekToTimestamp(percentageToTimestamp(percentPlayed, this.scrubber.start, this.scrubber.end));
  };

  @action
  seekToTimestamp = (timestamp: bigint) => {
    // Set the current time locally for quicker slider feedback.
    // When seeking we ignore timestamp updates from the server.
    this.scrubber.current = timestamp;
    this.seekDebounced();
  };

  /**
   * Debounce seek inputs to enforce only 1 active seek request on the network at a time per scrubber.
   */
  @action
  private seekDebounced = () => {
    if (this.scrubber.isSeeking) {
      return;
    }

    // Prevent more seek requests being sent out until we receive a response.
    // This also prevents the slider from being moved by updates from the server when we're awaiting a seek response.
    this.scrubber.isSeeking = true;

    // Cache the percent played from before the seek request is sent
    const timestamp = this.scrubber.current;

    // This checks if we need to send another seek request once the current request gets a response
    const onReceivedResponse = action((result: RpcResult<ScrubberSeekRequest_Response>) => {
      this.scrubber.isSeeking = false;

      // If the scrubber moved since sending the last request, send another for the new time
      if (result.ok && timestamp !== this.scrubber.current) {
        this.seekDebounced();
      } else {
        this.defaultRpcResultHandler(result);
      }
    });

    const request = new ScrubberSeekRequest({
      id: this.scrubber.id,
      timestamp: Timestamp.toMessage(timestamp),
    });
    this.network.call(request).then(onReceivedResponse);
  };

  private defaultRpcResultHandler = <T>(result: RpcResult<T>) => {
    if (!result.ok) {
      if (result.error.isRemoteError()) {
        this.onScrubberError(result.error.message);
      } else {
        result.error.defaultHandler();
      }
    }
  };

  private onScrubberError = (error: string, prefix?: string) => {
    // TODO: add toast messages to NUsight to show errors like this without using alert()
    alert(`${prefix ? prefix + ":\n" : ""}${error}`);
  };
}
