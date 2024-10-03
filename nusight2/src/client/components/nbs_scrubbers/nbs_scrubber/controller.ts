import { action } from "mobx";

import { message } from "../../../../shared/messages";
import { RpcResult } from "../../../../shared/messages/rpc_call";
import { TimestampObject } from "../../../../shared/time/timestamp";
import { RpcNetwork } from "../../../hooks/use_rpc_controller";
import { NbsScrubberModel } from "../model";
import { percentageToTimestamp } from "../util";

export class NbsScrubberController {
  constructor(private scrubber: NbsScrubberModel, private network: RpcNetwork) {}

  close = () => {
    this.network.call(new message.eye.ScrubberCloseRequest({ id: this.scrubber.id })).then((result) => {
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
    const request =
      this.scrubber.playbackState === "playing"
        ? new message.eye.ScrubberPauseRequest({ id: this.scrubber.id })
        : new message.eye.ScrubberPlayRequest({ id: this.scrubber.id });

    this.network.call(request).then(this.defaultRpcResultHandler);
  };

  playFaster = () => {
    const request = new message.eye.ScrubberSetPlaybackSpeedRequest({
      id: this.scrubber.id,
      playbackSpeed: this.scrubber.playbackSpeed + 1,
    });
    this.network.call(request).then(this.defaultRpcResultHandler);
  };

  playSlower = () => {
    const request = new message.eye.ScrubberSetPlaybackSpeedRequest({
      id: this.scrubber.id,
      playbackSpeed: this.scrubber.playbackSpeed - 1,
    });
    this.network.call(request).then(this.defaultRpcResultHandler);
  };

  toggleRepeat = () => {
    const request = new message.eye.ScrubberSetRepeatRequest({
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
    const onReceivedResponse = action((result: RpcResult<message.eye.ScrubberSeekRequest.Response>) => {
      this.scrubber.isSeeking = false;

      // If the scrubber moved since sending the last request, send another for the new time
      if (result.ok && timestamp !== this.scrubber.current) {
        this.seekDebounced();
      } else {
        this.defaultRpcResultHandler(result);
      }
    });

    const request = new message.eye.ScrubberSeekRequest({
      id: this.scrubber.id,
      timestamp: TimestampObject.fromNanos(timestamp),
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
