import { NUClearNetPacket } from "nuclearnet.js";

import { NUClearPacketMetadata } from "../../shared/nuclearnet/nuclearnet_client";
import { Clock } from "../../shared/time/clock";
import { NodeSystemClock } from "../time/node_clock";

import { decodePacketId } from "./decode_packet_id";
import { LruPriorityQueue } from "./lru_priority_queue";

type PacketSend = (event: string, ...args: any[]) => void;

/**
 * Processes NUClearNet packets being sent to a NUsight browser client using a LRU priority queue.
 * Avoids overwhelming the client with too many packets at a time, and one packet type
 * of a higher frequency dominating other packet types of lower frequency.
 *
 * Only "unreliable" packets are throttled, "reliable" packets are sent without queueing.
 */
export class NUClearNetPacketProcessor {
  /** The number of packets that have been sent, awaiting acknowledgement */
  private outgoingPackets: number = 0;

  /** The maximum number of packets to send before stopping to wait for acknowledgements */
  private readonly outgoingLimit: number;

  /** The number of seconds to wait before giving up on an acknowledgement */
  private readonly timeout: number;

  constructor(
    private send: PacketSend,
    private clock: Clock,
    private queue: LruPriorityQueue<string, { event: string; packet: NUClearNetPacket }>,
    { outgoingLimit, timeout }: { outgoingLimit: number; timeout: number },
  ) {
    this.outgoingLimit = outgoingLimit;
    this.timeout = timeout;
    this.queue = queue;
  }

  static of(send: PacketSend) {
    return new NUClearNetPacketProcessor(send, NodeSystemClock, new LruPriorityQueue({ capacityPerKey: 2 }), {
      outgoingLimit: 10,
      timeout: 5,
    });
  }

  /** Process a new packet of the given event type from NUClearNet */
  onPacket(event: string, packet: NUClearNetPacket, packetMetadata?: NUClearPacketMetadata) {
    // Always send reliable packets through without queueing
    if (packet.reliable) {
      this.send(event, packet);
    }
    // Throttle unreliable packets so that we do not overwhelm the client with traffic
    else {
      const [type] = event.split("#");
      const subtype = packetMetadata?.subtype ?? (decodePacketId(type, packet) as number);
      const key = `${type}#${subtype}:${packet.peer.name}:${packet.peer.address}:${packet.peer.port}`;

      this.queue.add(key, { event, packet });
      this.maybeSendNextPacket();
    }
  }

  /** Read from the queue and send the next packet if appropriate */
  private maybeSendNextPacket() {
    if (this.outgoingPackets < this.outgoingLimit) {
      const next = this.queue.pop();
      if (next) {
        const { event, packet } = next;

        let cancelAckTimeout: (() => void) | undefined = undefined;

        // For ignoring double calls (e.g. due to the client ack'ing multiple times)
        let doneAlreadyCalled = false;

        // Will be called when the client gets the message and acknowledges it,
        // or when the acknowledge timeout expires.
        const done = () => {
          if (doneAlreadyCalled) {
            return;
          }

          doneAlreadyCalled = true;

          cancelAckTimeout?.();

          this.outgoingPackets--;
          this.maybeSendNextPacket();
        };

        this.outgoingPackets++;

        cancelAckTimeout = this.clock.setTimeout(done, this.timeout);

        this.send(event, packet, done);
      }
    }
  }
}
