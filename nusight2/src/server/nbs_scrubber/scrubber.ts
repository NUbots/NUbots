import { NbsDecoder, NbsTimestamp, NbsTypeSubtypeBuffer } from "nbsdecoder.js";
import { NUClearNetPeer } from "nuclearnet.js";
import path from "path";

import { NbsScrubber } from "../../shared/nbs_scrubber";
import { Clock } from "../../shared/time/clock";
import { NodeSystemClock } from "../time/node_clock";

import { availableSyntheticTypes } from "./packet_synthesis";
import { timestampObjectToNanos } from "./utils";

/** Holds the data for a loaded NBS scrubber and handles interactions with the scrubber. */
export class Scrubber {
  /** The fake peer that the scrubber will be presented as */
  peer: NUClearNetPeer;

  /** NBS decoder with the files loaded for scrubbing */
  decoder: NbsDecoder;

  /** The types and subtypes available in the scrubber's files */
  availableTypes: NbsTypeSubtypeBuffer[];

  /** The core scrubber data, synced across clients in the same session */
  data: NbsScrubber;

  /** When the last playback started, in realtime */
  private playbackStartReal: number = 0;

  /** When the last playback started, in scrubber NBS time */
  private playbackStartNbs: bigint;

  /** The clock used for timing, configurable to aid testing */
  private clock: Clock;

  /** Whether this scrubber has been destroyed */
  private isDestroyed = false;

  /**
   * Create a scrubber with the given files, id, and optional name.
   * The name is what the scrubber will appear as in NUsight. A name will
   * be auto generated based on the file names if one is not provided.
   */
  constructor(files: string[], opts: { id: NbsScrubber["id"]; name?: string; clock: Clock }) {
    // Create a decoder for the files and set the available types to what's
    // in the files and what we can synthesize
    this.decoder = new NbsDecoder(files);
    this.availableTypes = [...this.decoder.getAvailableTypes(), ...availableSyntheticTypes()];

    // Get the timestamp range and set our playback start in NBS time
    const timestampRange = this.decoder.getTimestampRange();
    this.playbackStartNbs = timestampObjectToNanos(timestampRange[0]);

    // Set the core scrubber data
    this.data = {
      id: opts.id,
      name: opts.name ?? path.basename(files[0]) + (files.length > 1 ? ` (+ ${files.length - 1})` : ""),
      start: timestampRange[0],
      end: timestampRange[1],
      playbackRepeat: false,
      playbackSpeed: 0,
      playbackState: "paused",
    };

    // Create the fake peer that the scrubber will be presented as
    this.peer = {
      name: this.data.name,
      address: "0.0.0.0",
      port: opts.id,
    };

    this.clock = opts.clock;
  }

  /**
   * Create a scrubber with the given files, id, and optional name.
   * The name is what the scrubber will appear as in NUsight. A name will
   * be auto generated based on the file names if one is not provided.
   */
  static of(files: string[], opts: { id: NbsScrubber["id"]; name?: string; clock?: Clock }): Scrubber {
    return new Scrubber(files, {
      ...opts,
      clock: opts.clock ?? NodeSystemClock,
    });
  }

  /** The earliest timestamp in the decoder */
  get start(): bigint {
    return timestampObjectToNanos(this.data.start);
  }

  /** The latest timestamp in the decoder */
  get end(): bigint {
    return timestampObjectToNanos(this.data.end);
  }

  /** The total length of the scrubber, in nanoseconds */
  get length(): bigint {
    return this.end - this.start;
  }

  /** The current playback speed modifier */
  get playbackSpeedModifier(): number {
    return 2 ** this.data.playbackSpeed;
  }

  /**
   * Get the scrubber's current timestamp in nanoseconds. It's computed based on the current
   * playback state, repeat state, playback speed, and time elapsed since we started playing.
   */
  timestamp(): bigint {
    if (this.data.playbackState === "ended") {
      return this.end;
    }

    const realDelta = this.data.playbackState === "playing" ? (this.clock.now() - this.playbackStartReal) * 1e9 : 0;
    const nbsDelta = BigInt(Math.floor(realDelta * this.playbackSpeedModifier));

    let timestamp = this.playbackStartNbs + nbsDelta;

    // Check if we are past the end
    if (timestamp > this.end) {
      // Loop if we have repeat enabled
      if (this.data.playbackRepeat) {
        timestamp = this.start + ((timestamp - this.start) % this.length);
      }
      // End otherwise
      else {
        this.data.playbackState = "ended";
        timestamp = this.end;
      }
    }

    return timestamp;
  }

  /** Reset the playback start to the given timestamp */
  private resetPlaybackStart = (timestamp: bigint) => {
    this.playbackStartNbs = timestamp;
    this.playbackStartReal = this.clock.now();
  };

  /** Seek the scrubber to the given timestamp */
  seek = (timestamp: NbsTimestamp) => {
    // Switch to the `paused` state if we seek in the `ended` state.
    // This will prevent the slider from jumping back to the beginning when we play again.
    if (this.data.playbackState === "ended") {
      this.data.playbackState = "paused";
    }

    // Perform the seek
    this.resetPlaybackStart(timestampObjectToNanos(timestamp));
  };

  /** Start playback */
  play = () => {
    if (this.isDestroyed) {
      return;
    }

    if (this.data.playbackState === "ended") {
      this.resetPlaybackStart(this.start);
      this.data.playbackState = "playing";
    } else {
      this.resetPlaybackStart(this.timestamp());
      this.data.playbackState = "playing";
    }
  };

  /** Pause playback */
  pause = () => {
    if (this.isDestroyed) {
      return;
    }

    this.resetPlaybackStart(this.timestamp());
    this.data.playbackState = "paused";
  };

  /** Set the scrubber's repeat state */
  setRepeat = (repeat: boolean) => {
    this.resetPlaybackStart(this.timestamp());
    this.data.playbackRepeat = repeat;
  };

  /** Set the scrubber's playback speed. This will be used as a power of 2 to get the playback speed modifier. */
  setPlaybackSpeed = (speed: number) => {
    this.resetPlaybackStart(this.timestamp());
    this.data.playbackSpeed = speed;
  };

  /** Destroy the scrubber and prevent further interaction */
  destroy() {
    this.isDestroyed = true;
  }
}
