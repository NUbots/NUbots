import { NUClearNetPacket } from "nuclearnet.js";
import { NUClearNetPeer } from "nuclearnet.js";

import { MessageType } from "../../shared/messages";
import { Emit } from "../../shared/messages/emit";
import { messageTypeToName } from "../../shared/messages/type_converters";
import { hashType } from "../nuclearnet/hash_type";

import { NUsightSession } from "./session";
import { NUsightSessionClient } from "./session_client";

export type NUClearMessageCallback<T> = (peer: NUClearNetPeer, message: T) => void;
export type ClientMessageCallback<T> = (client: NUsightSessionClient, message: T) => void;

export type RpcResponse = { ok: true; rpcToken: number } | { ok: false; rpcToken: number; error: string };
export type RpcRequestHandler<T> = (request: T, sender: NUsightSessionClient) => RpcResponse | Promise<RpcResponse>;

/** Used for messages from the current NUsight server */
const NUSIGHT_SERVER_PEER = { name: "nusight", address: "0.0.0.0", port: 0 } as const;

/**
 * Handles networking in a NUsight session: allows for exchanging messages between clients in the session and NUClearNet.
 */
export class NUsightSessionNetwork {
  /** The session this network belongs to */
  private session: NUsightSession;

  /** Create a new network for the given session */
  constructor(session: NUsightSession) {
    this.session = session;
  }

  /**
   * Emit the given Protobuf message. The target is chosen based on `options.target` as follows:
   *   - "nusight" or `undefined` -> send to all clients in the session
   *   - "nusight#<id>"           -> send to the client with the given id
   *   - "*"                      -> send to all clients and NUClearNet
   *   - "" or any other string   -> send to NUClearNet
   */
  emit: Emit = (message, options = {}) => {
    const messageType = message.constructor as MessageType<any>;
    const messageTypeName = messageTypeToName(messageType);

    const payload = messageType.encode(message).finish() as Buffer;
    const hash = hashType(messageTypeName);
    const reliable = options.reliable ?? true;
    const peer = NUSIGHT_SERVER_PEER;

    if (options.target === undefined || options.target === "nusight") {
      const packet: NUClearNetPacket = { hash, reliable, payload, peer };
      this.session.sendToAll(messageTypeName, packet);
    } else if (options.target.startsWith("nusight#")) {
      const id = Number(options.target.replace(/^nusight#/, ""));
      const client = Array.from(this.session.clients).find((client) => client.id === id);

      if (!client) {
        throw new Error(`Could not emit to NUsight client with id ${id}, no such client`);
      }

      const packet: NUClearNetPacket = { hash, reliable, payload, peer };
      client.connection.send(messageTypeName, packet);
    } else if (options.target === "*") {
      const packet: NUClearNetPacket = { hash, reliable, payload, peer };
      this.session.sendToAll(messageTypeName, packet);
      this.session.nuclearnetClient.send({ type: hash, payload, reliable });
    } else {
      this.session.nuclearnetClient.send({ type: hash, payload, reliable, target: options.target });
    }
  };

  /** Register a listener for the given message type from NUClearNet */
  onNUClearMessage<T>(
    type: MessageType<T> | { type: MessageType<T>; subtype?: number },
    cb: NUClearMessageCallback<T>,
  ) {
    const { event, MessageType } = findMessageType(type);

    return this.session.nuclearnetClient.on(event, (packet: NUClearNetPacket) => {
      const buffer = new Uint8Array(packet.payload);
      const message = MessageType.decode(buffer);
      cb(packet.peer, message);
    });
  }

  /** Register a listener for the given message type from a client in the session */
  onClientMessage<T>(type: MessageType<T> | { type: MessageType<T>; subtype?: number }, cb: ClientMessageCallback<T>) {
    const { event, MessageType } = findMessageType(type);

    return this.session.onClientPacket(event, (client, packet) => {
      const buffer = new Uint8Array(packet.payload);
      const message = MessageType.decode(buffer);
      cb(client, message);
    });
  }

  /** Register a handler to respond to RPC requests of the given type from a client in the session */
  onClientRpc<T extends { rpcToken: number }>(
    type: MessageType<T> | { type: MessageType<T>; subtype?: number },
    requestHandler: RpcRequestHandler<T>,
  ) {
    return this.onClientMessage(type, async (client: NUsightSessionClient, request: T) => {
      try {
        const response = await requestHandler(request, client);
        this.emit(response as any, { target: `nusight#${client.id}` });
      } catch (error: unknown) {
        const ResponseType = (request.constructor as any).Response;
        this.emit(
          new ResponseType({
            ok: false,
            rpcToken: request.rpcToken,
            error: error instanceof Error ? error.message : String(error),
          }),
          { target: `nusight#${client.id}` },
        );
      }
    });
  }
}

/** Find the Protobuf message class and event name for the given type */
function findMessageType(type: MessageType<any> | { type: MessageType<any>; subtype?: number }) {
  const MessageType = typeof type === "object" ? type.type : type;
  const messageTypeName = messageTypeToName(MessageType);
  const subtype = typeof type === "object" ? type.subtype : null;

  return {
    event: `${messageTypeName}${subtype ? `#${subtype}` : ""}`,
    MessageType,
  };
}
