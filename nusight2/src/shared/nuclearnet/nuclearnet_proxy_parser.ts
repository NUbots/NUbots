import Emitter from "component-emitter";
import { NUClearNetPacket } from "nuclearnet.js";
import { NUClearNetSend } from "nuclearnet.js";
import { PacketType } from "socket.io-parser";
import { Packet } from "socket.io-parser";

/**
 * These encoder/decoders are used to improve the performance when transporting NUClearNet packets.
 * Most of these packets are composed of a few header fields and a large binary blob.
 * Other parsers such as the default one or msgpack parser do not do a good job of transporting these,
 * especially when they are large packets.
 * This is because in order to extract the data back into a usable format, msgpack must perform a buffer.slice()
 * to extract the bytes. If this is a large buffer, it causes a large memory copy that slows things down considerably.
 * The default one on the other hand does not handle binary data well.
 * This custom parser handles sending binary data much better as the resulting buffer is sent unmolested.
 * This means that at the other end that buffer can be directly used rather than slicing it.
 */

export class Encoder {
  encode(packet: Packet) {
    switch (packet.type) {
      case PacketType.EVENT:
      case PacketType.BINARY_EVENT: {
        const {
          nsp,
          data: [eventName],
        } = packet;

        switch (eventName) {
          // For our communication layer we can just use JSON
          case "nuclear_join":
          case "nuclear_leave":
          case "nuclear_connect":
          case "nuclear_disconnect":
          case "listen":
          case "unlisten":
            return [JSON.stringify(packet)];

          case "packet": {
            const {
              id,
              data: [key, { target, type, payload, reliable }],
            } = packet;
            return [JSON.stringify({ id, nsp, key, header: { target, type, reliable } }), payload];
          }

          // For NUClearNet packets, we send the payload separately to avoid array slicing later
          default: {
            const {
              id,
              data: [key, { peer, hash, payload, reliable }],
            } = packet;
            // Send the header as a JSON and then the payload as binary
            return [JSON.stringify({ id, nsp, key, header: { peer, reliable } }), hash, payload];
          }
        }
      }
      default:
        return [JSON.stringify(packet)];
    }
  }
}

export class Decoder extends Emitter {
  private state: number = 0;
  private nuclearPacket?: {
    nsp: string;
    type: PacketType.EVENT;
    data: [string, Partial<NUClearNetPacket> | Partial<NUClearNetSend>];
    id: number;
  };

  add(obj: any) {
    // Strings are json
    if (typeof obj === "string") {
      const parsed = JSON.parse(obj);

      // Parsed type exists on all the non NUClearNet packets
      if (parsed.type !== undefined) {
        // This was a jsonified packet
        this.emit("decoded", JSON.parse(obj));
        this.state = 0;
      } else {
        // This is a binary packet header
        this.nuclearPacket = {
          nsp: parsed.nsp,
          type: PacketType.EVENT,
          id: parsed.id,
          data: [parsed.key, parsed.header],
        };
        this.state = 1;
      }
    } else {
      switch (this.state) {
        // State 1 means we are getting a payload or hash
        case 1:
          if (this.nuclearPacket!.data[0] === "packet") {
            // 'packet' messages are sent using NUClearNetSend which doesn't have a hash,
            // so here we get the payload and emit
            this.nuclearPacket!.data[1].payload = obj;
            this.emit("decoded", this.nuclearPacket);
            this.state = 0;
          } else {
            // For NUClearNetPackets we get a hash in state 1 and move to state 2 for the payload
            (this.nuclearPacket!.data[1] as NUClearNetPacket).hash = obj;
            this.state = 2;
          }
          break;

        // State 2 means we are getting a packet
        case 2:
          this.nuclearPacket!.data[1].payload = obj;
          this.emit("decoded", this.nuclearPacket);
          this.state = 0;
          break;

        // Something went wrong, reset
        default:
          this.state = 0;
          break;
      }
    }
  }

  destroy() {
    this.nuclearPacket = undefined;
  }
}
