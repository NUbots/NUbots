import { NUClearNetPacket } from "nuclearnet.js";
import { NUClearNetPeer } from "nuclearnet.js";
import { NUClearNetOptions } from "nuclearnet.js";
import { NUClearNetSend } from "nuclearnet.js";

export type NUClearPacketMetadata = { subtype: number };

export type NUClearPacketListener = (packet: NUClearNetPacket, packetMetadata?: NUClearPacketMetadata) => void;

export type NUClearEventListener = (peer: NUClearNetPeer) => void;

export interface NUClearNetClient {
  /** Connect to the NUClear network */
  connect(options: NUClearNetOptions): () => void;

  /** Register a callback to be called when a peer joins the network */
  onJoin(cb: NUClearEventListener): () => void;

  /** Register a callback to be called when a peer leaves the network */
  onLeave(cb: NUClearEventListener): () => void;

  /**
   * The `event` parameter is a string that can be in one of two formats:
   *   - `type` - e.g. "message.input.Image" for all images
   *   - `type#subtype` - e.g. "message.input.Image#1" for images where the subtype (i.e. `id` field) is 1.
   *     For events in this format, the callback on the server will be called with an additional argument
   *     `packetMetadata`, which will contain the decoded subtype of the packet.
   */
  on(event: string, cb: NUClearPacketListener): () => void;

  /** Register a callback to be called when a packet is received */
  onPacket(cb: NUClearPacketListener): () => void;

  /** Send a packet to the NUClear network */
  send(options: NUClearNetSend): void;
}

type PartialBy<T, K extends keyof T> = Omit<T, K> & Partial<Pick<T, K>>;

/** A data packet received from the NUClear network, where the payload is possibly undefined */
export type NUClearNetPacketMaybeEmpty = PartialBy<NUClearNetPacket, "payload">;
