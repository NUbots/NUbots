import { CancelTimer, Clock } from "../../shared/time/clock";
import { NodeSystemClock } from "../time/node_clock";

import { ScrubberPacketListenerCallback, ScrubberSet } from "./scrubber_set";

/** Callback for sending packets to a NUsight browser client */
export type PacketSend = (event: string, ...args: any[]) => void;

const logScrubberTimings = process.argv.includes("--log-scrubber-timings");

/**
 * Provides a processor for listening for messages from an NBS scrubber set and sending
 * them to a client, with a Promise-wrapper on client acks and timeouts as appropriate
 */
export class NbsPacketProcessor {
  /** The set of scrubbers to listen for packets from */
  private readonly scrubberSet: ScrubberSet;

  /** Callback for sending packets to the client  */
  private readonly send: PacketSend;

  /** The clock used for timing, configurable to aid testing */
  private readonly clock: Clock;

  /** Clean up callbacks for currently registered events */
  private onEventsCleanUps: Set<() => void> = new Set();

  /** How long to wait for a sent packet to be acknowledged before giving up */
  private ackTimeout: number;

  /** Cancel callbacks for timers of currently pending acks. Maps a timer's cancel callback to a clean up function. */
  private pendingAckTimers: Map<CancelTimer, () => void> = new Map();

  constructor(scrubberSet: ScrubberSet, send: PacketSend, opts: { clock: Clock; ackTimeout: number }) {
    this.scrubberSet = scrubberSet;
    this.send = send;
    this.clock = opts.clock;
    this.ackTimeout = opts.ackTimeout;
  }

  static of(scrubberSet: ScrubberSet, send: PacketSend, opts: { clock?: Clock; ackTimeout?: number } = {}) {
    return new NbsPacketProcessor(scrubberSet, send, {
      clock: opts.clock ?? NodeSystemClock,
      ackTimeout: opts.ackTimeout ?? 2000,
    });
  }

  /** Listen for the given event from the scrubber set for forwarding to the client */
  listenFor(event: string): () => void {
    const callback: ScrubberPacketListenerCallback = (packet) => {
      return new Promise((resolve) => {
        let sentAt = 0;
        let alreadyResolved = false;
        let cancelAckTimeout: CancelTimer | undefined = undefined;

        // Create the callback for when the client acknowledges a sent packet
        const onClientDone = (clientAcknowledgedAt: number, clientReceivedAt: number) => {
          if (cancelAckTimeout) {
            cancelAckTimeout?.();
            this.pendingAckTimers.delete(cancelAckTimeout);
            cancelAckTimeout = undefined;
          }

          if (alreadyResolved) {
            return;
          }

          alreadyResolved = true;

          const now = Date.now();

          if (logScrubberTimings) {
            // Note: the following time calculations assume the client and server have clocks that are
            // synchronized to within a few milliseconds. This will be true when running the server
            // and client on the same machine, but may not be exactly true when running otherwise.
            // In the latter case these timings should be taken as only rough approximations.
            console.log(`scrubber request for ${event}`, {
              serverToClient: clientReceivedAt - sentAt,
              clientProcessing: clientAcknowledgedAt - clientReceivedAt,
              clientBackToServer: now - clientAcknowledgedAt,
              total: now - sentAt,
            });
          }

          resolve();
        };

        // Start a timer to give up on the ack if it's not received in the configured timeout
        cancelAckTimeout = this.clock.setTimeout(() => {
          if (cancelAckTimeout) {
            this.pendingAckTimers.delete(cancelAckTimeout);
            cancelAckTimeout = undefined;
          }

          if (alreadyResolved) {
            return;
          }

          alreadyResolved = true;

          resolve();
        }, this.ackTimeout);

        // Register a callback so we can cancel the ack timer if the processor is destroyed while waiting
        this.pendingAckTimers.set(cancelAckTimeout, () => {
          if (cancelAckTimeout) {
            cancelAckTimeout();
            this.pendingAckTimers.delete(cancelAckTimeout);
            cancelAckTimeout = undefined;
          }

          if (alreadyResolved) {
            return;
          }

          alreadyResolved = true;

          resolve();
        });

        // Keep track of when we sent the packet
        sentAt = Date.now();

        // Send the packet
        this.send(event, packet, onClientDone);
      });
    };

    // Listen for the event on the scrubber set
    this.scrubberSet.on(event, callback);

    // Create a clean up function for removing the event listener
    const cleanUp = () => {
      this.scrubberSet.off(event, callback);
      this.onEventsCleanUps.delete(cleanUp);
    };

    this.onEventsCleanUps.add(cleanUp);

    return cleanUp;
  }

  /** Close the packet processor, removing all event listeners and cancelling all pending ack timeouts */
  destroy() {
    // Clean up (and remove) all event listeners
    for (const cleanUp of this.onEventsCleanUps) {
      cleanUp();
    }

    // Cancel all ack timers
    for (const cancel of this.pendingAckTimers.values()) {
      cancel();
    }

    // Clear the ack timers
    this.pendingAckTimers.clear();
  }
}
