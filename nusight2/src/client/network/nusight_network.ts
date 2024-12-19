import { runInAction } from "mobx";
import { NUClearNetPacket } from "nuclearnet.js";
import { NUClearNetOptions } from "nuclearnet.js";
import { NUClearNetSend } from "nuclearnet.js";

import { MessageType } from "../../shared/messages";
import { Emit } from "../../shared/messages/emit";
import { messageTypeToName } from "../../shared/messages/type_converters";
import { NUClearNetClient, NUClearNetPeerWithType } from "../../shared/nuclearnet/nuclearnet_client";
import { memoize } from "../base/memoize";
import { AppModel } from "../components/app/model";
import { RobotModel } from "../components/robot/model";
import { WebSocketClient } from "../nuclearnet/web_socket_client";
import { WebSocketProxyNUClearNetClient } from "../nuclearnet/web_socket_proxy_nuclearnet_client";

import { RobotNetworkStatsModel } from "./model";

/**
 * This static robot model is used for protobuf messages coming from the NUsight server.
 * This is necessary since the client networking setup requires an existing and
 * connected RobotModel instance for every peer we receive a protobuf message from.
 */
const nusightServerRobotModel = RobotModel.of({
  name: "nusight",
  address: "0.0.0.0",
  connected: true,
  enabled: true,
  id: "nusight",
  port: 0,
  type: "nusight-server",
});

/**
 * This class is intended to handle NUsight-specific networking. It handles the subscription of NUClearNet messages and
 * decoding them into real protobufjs objects for convenient use. Components should not directly use this class, but
 * instead create their own ComponentNetwork class which uses the Network helper class.
 */
export class NUsightNetwork {
  constructor(
    private nuclearnetClient: NUClearNetClient,
    private appModel: AppModel,
  ) {}

  static of = memoize((appModel: AppModel) => {
    const nuclearnetClient: NUClearNetClient = WebSocketProxyNUClearNetClient.of();
    return new NUsightNetwork(nuclearnetClient, appModel);
  });

  connect(opts: NUClearNetOptions): () => void {
    return this.nuclearnetClient.connect(opts);
  }

  send(opts: NUClearNetSend) {
    this.nuclearnetClient.send(opts);
  }

  emit: Emit = (message, opts) => {
    const type = Object.getPrototypeOf(message).constructor;
    const messageTypeName = messageTypeToName(type);
    const payload = type.encode(message).finish();

    this.send({
      type: messageTypeName,
      payload: payload as Buffer,
      reliable: opts?.reliable ?? false,
      target: opts?.target,
    });
  };

  /**
   * Provide the given callback access to the underlying websocket transport
   */
  useSocket(cb: (socket: WebSocketClient) => void) {
    if (this.nuclearnetClient instanceof WebSocketProxyNUClearNetClient) {
      this.nuclearnetClient.useSocket(cb);
    }
  }

  onNUClearMessage<T>(type: MessageType<T> | { type: MessageType<T>; subtype?: number }, cb: MessageCallback<T>) {
    const messageType = typeof type === "object" ? type.type : type;
    const messageTypeName = messageTypeToName(messageType);
    const subtype = typeof type === "object" ? type.subtype : null;

    const event = `${messageTypeName}${subtype ? `#${subtype}` : ""}`;

    return this.nuclearnetClient.on(event, (packet: NUClearNetPacket) => {
      return new Promise<void>((resolve) => {
        runInAction(async () => {
          try {
            const buffer = new Uint8Array(packet.payload);
            const message = messageType.decode(buffer);
            const peer = packet.peer;
            const robotModel = this.appModel.robots.find((robot) => {
              return robot.name === peer.name && robot.address === peer.address && robot.port === peer.port;
            });
            if (robotModel) {
              const stats = RobotNetworkStatsModel.of(robotModel);
              stats.packets += 1;
              stats.packetsPerSecond.update(1);
              stats.bytes += buffer.length;
              stats.bytesPerSecond.update(buffer.length);
              await cb(robotModel, message);
            } else if (peer.name === "nusight") {
              // For messages from the NUsight server, we use a known static robot model
              await cb(nusightServerRobotModel, message);
            }
          } finally {
            resolve();
          }
        });
      });
    });
  }

  onNUClearJoin(cb: (peer: NUClearNetPeerWithType) => void) {
    this.nuclearnetClient.onJoin(cb);
  }

  onNUClearLeave(cb: (peer: NUClearNetPeerWithType) => void) {
    this.nuclearnetClient.onLeave(cb);
  }
}

export type MessageCallback<T> = (robotModel: RobotModel, message: T) => void;
