import { NbsTimestamp } from "nbsdecoder.js";

import { NbsScrubber } from "../../shared/nbs_scrubber";
import { NUClearNetPacketMaybeEmpty } from "../../shared/nuclearnet/nuclearnet_client";
import { Clock } from "../../shared/time/clock";
import { hashType } from "../nuclearnet/hash_type";
import { parseEventString } from "../nuclearnet/parse_event_string";
import { NodeSystemClock } from "../time/node_clock";

import { isSynthesizable, synthesize } from "./packet_synthesis";
import { Scrubber } from "./scrubber";

export type ScrubberUpdateOpts =
  | {
      id: NbsScrubber["id"];
      type: "play";
    }
  | {
      id: NbsScrubber["id"];
      type: "pause";
    }
  | {
      id: NbsScrubber["id"];
      type: "set-playback-speed";
      playbackSpeed: number;
    }
  | {
      id: NbsScrubber["id"];
      type: "set-repeat";
      repeat: boolean;
    }
  | {
      id: NbsScrubber["id"];
      type: "seek";
      timestamp: NbsTimestamp;
    };

/** A callback function that can listen for packets from the scrubbers */
export type ScrubberPacketListenerCallback = (packet: NUClearNetPacketMaybeEmpty) => Promise<void>;

/**
 * An individual packet listener in the scrubber set. Each listener is
 * a unique combination of type, subtype, callback, and scrubber.
 */
export interface ScrubberPacketListener {
  /** The type this listener is for */
  type: Buffer;

  /** The subtype this listener is for */
  subtype: number;

  /** The callback function for this listener */
  callback: ScrubberPacketListenerCallback;

  /** The scrubber this listener is attached to */
  scrubber: Scrubber;

  /** A promise that resolves when the listener's callback is done processing a packet */
  pendingPromise?: Promise<void>;

  /** Whether this listener has been removed from the scrubber set */
  removed?: boolean;
}

/** Represents an event registration for a callback in the scrubber set */
interface ScrubberCallbackRegistration {
  /** The event string passed to `on()` during registration */
  event: string;

  /** The type, as parsed from the event string */
  type: Buffer;

  /** The subtype, as parsed from the event string. Undefined when no subtype is specified in the event string. */
  subtype?: number;

  /** The callback passed ton `on()` during registration. */
  callback: ScrubberPacketListenerCallback;

  /** A map of listeners for this callback, keyed by the scrubber they are attached to. */
  listenersByScrubber: Map<Scrubber, Set<ScrubberPacketListener>>;
}

/**
 * ScrubberSet holds the set of all scrubbers loaded in a given session. It provides an interface
 * for packet event listeners to be registered on the set as a whole, allowing callers to
 * remain unaware of the individual scrubbers and their inner workings. When a scrubber in
 * the set is seeked or changed, the scrubber set will emit the appropriate packets to the
 * right listeners, throttling the packet emission per listener to avoid flooding the
 * client with packets.
 */
export class ScrubberSet {
  /** The scrubbers currently loaded in the set. Maps scrubber IDs to Scrubber instances. */
  scrubbers: Map<NbsScrubber["id"], Scrubber> = new Map();

  /** The registered callbacks in the set. Maps callbacks to their registrations. */
  callbacks: Map<ScrubberPacketListenerCallback, ScrubberCallbackRegistration> = new Map();

  /** The clock used for timing, configurable to aid testing */
  clock: Clock;

  constructor(opts: { clock: Clock }) {
    this.clock = opts.clock;
  }

  static of(opts?: { clock?: Clock }) {
    return new ScrubberSet({
      clock: opts?.clock ?? NodeSystemClock,
    });
  }

  /** Register a listener for the given event on the scrubber set. */
  on(event: string, callback: ScrubberPacketListenerCallback) {
    if (this.callbacks.has(callback)) {
      throw new Error(`Cannot register the same callback twice: event ${event}`);
    }

    // Parse the event string into a type and subtype, and compute the type's hash
    const { type: typeStr, subtype } = parseEventString(event, { throwOnInvalidSubtype: false });
    const type = hashType(typeStr);

    const registration: ScrubberCallbackRegistration = {
      event,
      type,
      subtype,
      callback,
      listenersByScrubber: new Map(),
    };
    this.callbacks.set(callback, registration);

    // Register scrubber-specific listeners for the new callback
    for (const scrubber of this.scrubbers.values()) {
      const listeners = this.registerListeners(scrubber, callback, type, subtype);
      registration.listenersByScrubber.set(scrubber, listeners);
    }
  }

  /** Register listeners for all types in the given scrubber that match the given type + subtype */
  private registerListeners(
    scrubber: Scrubber,
    callback: ScrubberPacketListenerCallback,
    type: Buffer,
    subtype?: number,
  ) {
    // Find all available types in the scrubber that match the given type and subtype
    const matchingTypeSubtypes = scrubber.availableTypes.filter(
      (available) => available.type.equals(type) && (subtype === undefined || available.subtype === subtype),
    );

    // Create a listener for each matching type + subtype
    const listeners = matchingTypeSubtypes.map((available) => {
      const listener: ScrubberPacketListener = {
        type,
        subtype: available.subtype,
        callback,
        scrubber,
        removed: false,
        pendingPromise: undefined,
      };

      // On registration, we emit to the listener
      this.loadAndEmitToListener(listener, scrubber.timestamp());

      return listener;
    });

    return new Set(listeners);
  }

  /** Remove a listener for the given event from the scrubber set */
  off(event: string, callback: ScrubberPacketListenerCallback) {
    const registration = this.callbacks.get(callback);

    if (registration) {
      for (const listeners of registration.listenersByScrubber.values()) {
        for (const listener of listeners) {
          if (listener.pendingPromise) {
            // If there is a pending promise, wait for it to resolve and then remove the listener
            listener.removed = true;
            listener.pendingPromise.then(() => listeners.delete(listener));
          } else {
            // If there is no pending promise, remove the listener immediately
            listeners.delete(listener);
          }
        }
      }

      this.callbacks.delete(callback);
    }
  }

  /** Get the scrubber with the given ID from this set */
  get(id: NbsScrubber["id"]) {
    return this.scrubbers.get(id);
  }

  /** Load a new scrubber into the set, with the given NBS files and optional name */
  load(opts: { files: string[]; name?: string; onCreate?: (scrubber: Scrubber) => void }) {
    // Generate a unique ID and create the scrubber
    const id = this.scrubbers.size + 1;
    const scrubber = Scrubber.of(opts.files, { id, name: opts.name, clock: this.clock });

    // Add the scrubber to the set
    this.scrubbers.set(id, scrubber);

    // Run the onCreate callback, if provided, before registering listeners
    opts.onCreate?.(scrubber);

    // Register listeners on the new scrubber for all current callbacks that match the scrubber's types
    for (const { type, subtype, callback, listenersByScrubber } of this.callbacks.values()) {
      const listeners = this.registerListeners(scrubber, callback, type, subtype);
      listenersByScrubber.set(scrubber, listeners);
    }

    return scrubber;
  }

  /** Handle a request from the client to update a scrubber (seek, toggle playback, etc) */
  update(request: ScrubberUpdateOpts) {
    const scrubber = this.scrubbers.get(request.id);

    if (!scrubber) {
      throw new Error(`scrubber ${request.id} not found for update (${request.type})`);
    }

    switch (request.type) {
      case "seek":
        scrubber.seek(request.timestamp);
        break;

      case "play":
        scrubber.play();
        break;

      case "pause":
        scrubber.pause();
        break;

      case "set-repeat":
        scrubber.setRepeat(request.repeat);
        break;

      case "set-playback-speed":
        scrubber.setPlaybackSpeed(request.playbackSpeed);
        break;
    }

    this.processScrubberChange(scrubber, scrubber.timestamp());
  }

  /** Close and clean up the scrubber with the given ID and return it */
  close(id: NbsScrubber["id"]) {
    const scrubber = this.scrubbers.get(id);

    if (!scrubber) {
      throw new Error(`scrubber ${id} not found for close`);
    }

    // Remove all listeners for the scrubber
    for (const registration of this.callbacks.values()) {
      const listeners = registration.listenersByScrubber.get(scrubber) ?? [];
      for (const listener of listeners) {
        listener.removed = true;
      }
      registration.listenersByScrubber.delete(scrubber);
    }

    // Close the scrubber
    scrubber.destroy();

    // Remove the scrubber from the set
    this.scrubbers.delete(id);

    return scrubber;
  }

  /** Close and clean up all scrubbers and event listeners in this set */
  destroy() {
    // Close all scrubbers (which removes all listeners)
    for (const scrubber of this.scrubbers.values()) {
      this.close(scrubber.data.id);
    }

    // Remove all callbacks
    this.callbacks.clear();
  }

  /**
   * Process a change to the given scrubber at the given timestamp, by loading
   * and emitting the any packets at that timestamp in the scrubber.
   */
  private processScrubberChange(scrubber: Scrubber, timestamp: bigint) {
    for (const { listenersByScrubber } of this.callbacks.values()) {
      const listeners = listenersByScrubber.get(scrubber) ?? [];
      for (const listener of listeners) {
        this.loadAndEmitToListener(listener, timestamp);
      }
    }
  }

  /**
   * Load and emit the matching packet for the given listener at the given timestamp,
   * if the listener is not already processing a packet.
   */
  private loadAndEmitToListener(listener: ScrubberPacketListener, timestamp: bigint) {
    // Abort if the listener is already processing a packet. This function will
    // be called again when the listener is done processing the packet.
    if (listener.pendingPromise) {
      return;
    }

    const scrubber = listener.scrubber;
    const typeSubtype = { type: listener.type, subtype: listener.subtype };

    const typeIsSynthesizable = isSynthesizable(typeSubtype);

    // Read the NBS packet from the scrubber's decoder, or synthesize it
    const nbsPacket = typeIsSynthesizable
      ? synthesize(typeSubtype, listener.scrubber, timestamp)
      : scrubber.decoder.getPackets(timestamp, [typeSubtype])[0];

    // Create the NUClearNet packet that NUsight expects
    const packet: NUClearNetPacketMaybeEmpty = {
      hash: nbsPacket.type,
      payload: nbsPacket.payload,
      peer: scrubber.peer,
      reliable: true,
    };

    // Serialize the scrubber's state at the time the packet is being sent, for later comparison
    const stateAtSend = JSON.stringify(scrubber.data);

    const onProcessingDone = () => {
      // Clear the pending promise so that `loadAndEmitPacket()` can be called again
      listener.pendingPromise = undefined;

      // Abort if the listener was removed while it was processing the packet
      if (listener.removed) {
        return;
      }

      const newTimestamp = scrubber.timestamp();
      const newState = JSON.stringify(scrubber.data);

      // If the scrubber's state or timestamp is different from when we sent the last packet,
      // then load and emit a packet at the scrubber's current timestamp. We also emit a packet
      // if the scrubber is playing, regardless of a change in state or timestamp, to avoid
      // breaking the server send + client ack cycle during playback.
      if (
        newTimestamp !== timestamp ||
        scrubber.data.playbackState === "playing" ||
        (typeIsSynthesizable && newState !== stateAtSend)
      ) {
        this.loadAndEmitToListener(listener, newTimestamp);
      }
    };

    // Emit the packet to the listener, and call the handler above when the listener is done processing
    listener.pendingPromise = listener.callback(packet).then(onProcessingDone);
  }
}
