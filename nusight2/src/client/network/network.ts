import { NUClearNetSend } from "nuclearnet.js";

import { MessageType } from "../../shared/messages";
import { Emit } from "../../shared/messages/emit";
import { WebSocketClient } from "../nuclearnet/web_socket_client";

import { NUsightNetwork } from "./nusight_network";
import { MessageCallback } from "./nusight_network";
import { RpcClient } from "./rpc_client";

/**
 * A convenience helper class to be used at the component-level.
 *
 * Easily subscribe to multiple NUClearNet messages with on(MessageType) and then unsubscribe from them all with off().
 */
export class Network {
  // Store all event listener removers, so we can call them all in off().
  private offNUClearMessages: Set<() => void>;

  /** The client used to make RPC calls */
  private rpcClient: RpcClient;

  call: typeof RpcClient.prototype.call;

  constructor(private nusightNetwork: NUsightNetwork) {
    this.offNUClearMessages = new Set();
    this.rpcClient = new RpcClient(this);
    this.call = this.rpcClient.call.bind(this.rpcClient);
  }

  static of(nusightNetwork: NUsightNetwork): Network {
    return new Network(nusightNetwork);
  }

  /**
   * Subscribe to a NUClearNet message.
   *
   * @param type The protobuf message type that a robot would send. e.g. message.input.Sensors,
   *             or an object with a type and subtype.
   * @param cb The callback to call every time a message is received
   * @returns A unsubscriber function.
   */
  on<T>(type: MessageType<T> | { type: MessageType<T>; subtype?: number }, cb: MessageCallback<T>): () => void {
    const offNUClearMessage = this.nusightNetwork.onNUClearMessage(type, cb);
    this.offNUClearMessages.add(offNUClearMessage);
    return () => {
      offNUClearMessage();
      this.offNUClearMessages.delete(offNUClearMessage);
    };
  }

  /**
   * Unsubscribe from all events that is currently being listened to.
   */
  off() {
    this.rpcClient.cancelAll();
    for (const offNUClearMessage of this.offNUClearMessages.values()) {
      offNUClearMessage();
    }
    this.offNUClearMessages.clear();
  }

  /**
   * Send the given message on the network
   */
  send(opts: NUClearNetSend): void {
    this.nusightNetwork.send(opts);
  }

  /** Emit the given message on the network */
  emit: Emit = (message, options) => {
    this.nusightNetwork.emit(message, options);
  };

  /**
   * Provide the given callback access to the underlying websocket transport
   */
  useSocket(cb: (socket: WebSocketClient) => void) {
    this.nusightNetwork.useSocket(cb);
  }
}
