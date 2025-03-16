import { compose } from "../../shared/base/compose";
import { message } from "../../shared/messages";
import { NUsightSession } from "../session/session";

import ScrubberLoadRequest = message.eye.ScrubberLoadRequest;
import ScrubberCloseRequest = message.eye.ScrubberCloseRequest;
import ScrubberPlayRequest = message.eye.ScrubberPlayRequest;
import ScrubberPauseRequest = message.eye.ScrubberPauseRequest;
import ScrubberSetPlaybackSpeedRequest = message.eye.ScrubberSetPlaybackSpeedRequest;
import ScrubberSetRepeatRequest = message.eye.ScrubberSetRepeatRequest;
import ScrubberSeekRequest = message.eye.ScrubberSeekRequest;
import ScrubberClosed = message.eye.ScrubberClosed;

/** Handles server-side networking for the NBS scrubber by responding to RPC calls from the scrubber client */
export class NbsScrubberNetwork {
  /** Cleans up by removing all event listeners registered in this class */
  private cleanUp?: () => void;

  constructor(private session: NUsightSession) {
    const network = session.network;
    this.cleanUp = compose([
      network.onClientRpc(ScrubberLoadRequest, this.onLoad),
      network.onClientRpc(ScrubberCloseRequest, this.onClose),
      network.onClientRpc(ScrubberPlayRequest, this.onPlay),
      network.onClientRpc(ScrubberPauseRequest, this.onPause),
      network.onClientRpc(ScrubberSetPlaybackSpeedRequest, this.onSetPlaybackSpeed),
      network.onClientRpc(ScrubberSetRepeatRequest, this.onSetRepeat),
      network.onClientRpc(ScrubberSeekRequest, this.onSeek),
    ]);
  }

  /** Destroy the network by removing all event listeners */
  public destroy() {
    if (this.cleanUp) {
      this.cleanUp();
      this.cleanUp = undefined;
    }
  }

  /** Handle a request from the client to load an NBS scrubber */
  private onLoad = (request: ScrubberLoadRequest) => {
    const scrubber = this.session.scrubberSet.load({
      name: request.name ? request.name : undefined,
      files: request.files!,
      onCreate: (scrubber) => {
        this.session.sendToAll("nuclear_join", scrubber.peer);
      },
    });

    console.info(`Scrubber loaded: ${scrubber.data.name} (${scrubber.data.id})`, request.files);

    return new ScrubberLoadRequest.Response({ rpc: { ok: true, token: request.rpc?.token } });
  };

  /** Handle a request from the client to play a scrubber */
  private onPlay = (request: ScrubberPlayRequest) => {
    this.session.scrubberSet.update({ type: "play", id: request.id });

    return new ScrubberPlayRequest.Response({ rpc: { ok: true, token: request.rpc?.token } });
  };

  /** Handle a request from the client to pause a scrubber */
  private onPause = (request: ScrubberPauseRequest) => {
    this.session.scrubberSet.update({ type: "pause", id: request.id });

    return new ScrubberPauseRequest.Response({ rpc: { ok: true, token: request.rpc?.token } });
  };

  /** Handle a request from the client to set the playback speed of a scrubber */
  private onSetPlaybackSpeed = (request: ScrubberSetPlaybackSpeedRequest) => {
    this.session.scrubberSet.update({
      type: "set-playback-speed",
      id: request.id,
      playbackSpeed: request.playbackSpeed,
    });

    return new ScrubberSetPlaybackSpeedRequest.Response({ rpc: { ok: true, token: request.rpc?.token } });
  };

  /** Handle a request from the client to set the repeat mode of a scrubber */
  private onSetRepeat = (request: ScrubberSetRepeatRequest) => {
    this.session.scrubberSet.update({
      type: "set-repeat",
      id: request.id,
      repeat: request.repeat,
    });

    return new ScrubberSetRepeatRequest.Response({ rpc: { ok: true, token: request.rpc?.token } });
  };

  /** Handle a request from the client to seek a scrubber */
  private onSeek = (request: ScrubberSeekRequest) => {
    this.session.scrubberSet.update({
      type: "seek",
      id: request.id,
      timestamp: {
        seconds: request.timestamp!.seconds!,
        nanos: request.timestamp!.nanos!,
      },
    });

    return new ScrubberSeekRequest.Response({ rpc: { ok: true, token: request.rpc?.token } });
  };

  /** Handle a request from the client to close a scrubber */
  private onClose = (request: ScrubberCloseRequest) => {
    const scrubber = this.session.scrubberSet.close(request.id);

    // Remove the scrubber's fake peer from the network
    // TODO: use the `peer.type` property which indicates that the peer leaving
    // is a scrubber to avoid the need for the extra message below
    this.session.sendToAll("nuclear_leave", scrubber.peer);

    // Remove the scrubber from all browser clients in the session
    this.session.network.emit(new ScrubberClosed({ id: request.id }), {
      target: "nusight", // send to all clients in the session
    });

    console.info(`Scrubber closed: ${scrubber.data.name} (${scrubber.data.id})`);

    return new ScrubberCloseRequest.Response({ rpc: { ok: true, token: request.rpc?.token } });
  };
}
