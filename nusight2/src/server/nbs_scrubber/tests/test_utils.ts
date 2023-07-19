import { NbsTimestamp } from "nbsdecoder.js";
import path from "path";

import { message } from "../../../shared/messages";
import { NbsScrubber } from "../../../shared/nbs_scrubber";
import { NUClearNetPacketMaybeEmpty } from "../../../shared/nuclearnet/nuclearnet_client";
import { hashType } from "../../nuclearnet/hash_type";

import ScrubberState = message.eye.ScrubberState;

/** A default peer name used for scrubber in tests */
export const scrubberPeerName = "myScrubber";

// See the README in nbs_samples directory for what's in the samples
export const samplesDir = path.join(__dirname, "nbs_samples");
export const sampleFileA = path.join(__dirname, "nbs_samples", "sample-000-300.nbs");
export const sampleFileB = path.join(__dirname, "nbs_samples", "sample-300-600.nbs");

/** Get the default the scrubber state produced when loading sampleFileA */
export function sampleADefaultState(name = scrubberPeerName): NbsScrubber {
  return {
    id: 1,
    start: { seconds: 1000, nanos: 0 },
    end: { seconds: 1299, nanos: 0 },
    name,
    playbackRepeat: false,
    playbackSpeed: 0,
    playbackState: "paused",
  };
}

/** Make a NUClearNet packet of the given type and payload */
export function makePacket(
  typeName: "message.Ping" | "message.Pong" | "message.Pang",
  payload?: string,
  name = scrubberPeerName,
): NUClearNetPacketMaybeEmpty {
  const peer = {
    name,
    address: "127.0.0.1",
    port: 0,
  };

  const packet = {
    hash: hashType(typeName),
    payload: payload ? Buffer.from(payload) : undefined,
    reliable: true,
    peer,
  };

  return packet;
}

const ScrubberStateStringToEnum = {
  paused: ScrubberState.State.PAUSED,
  playing: ScrubberState.State.PLAYING,
  ended: ScrubberState.State.ENDED,
  unknown: ScrubberState.State.UNKNOWN,
} as const;

/** Make a NUClearNet packet of type ScrubberState with the given data */
export function makeScrubberStatePacket(
  opts: Partial<NbsScrubber> & {
    id: NbsScrubber["id"];
    timestamp: NbsTimestamp;
    start: NbsTimestamp;
    end: NbsTimestamp;
    playbackState: NbsScrubber["playbackState"];
    peer?: { name: string; address: string; port: number };
  },
): NUClearNetPacketMaybeEmpty {
  const {
    id,
    name = scrubberPeerName,
    timestamp,
    start,
    end,
    playbackState,
    playbackRepeat = false,
    playbackSpeed = 0,
  } = opts;

  const peer = opts.peer ?? {
    name,
    address: "127.0.0.1",
    port: 0,
  };

  const packet = {
    hash: hashType("message.eye.ScrubberState"),
    payload: Buffer.from(
      ScrubberState.encode({
        id,
        name,
        timestamp,
        start,
        end,
        playbackState: ScrubberStateStringToEnum[playbackState],
        playbackRepeat,
        playbackSpeed,
      }).finish(),
    ),
    reliable: true,
    peer,
  };

  return packet;
}

/** Compute the timestamp for the nth set of messages in the sample NBS files */
export function computeTimestampForMessageSet(set: number, offsetSeconds = 2): NbsTimestamp {
  const start = 1000; // Timestamps in the sample files start at 1000 seconds
  const setLength = 3; // 3 messages per set in the sample file

  return {
    seconds: start + set * setLength + offsetSeconds,
    nanos: 0,
  };
}

/** Compute that nth set of messages at the given timestamp in the sample NBS files */
export function computeMessageSetForTimestamp(timestamp: NbsTimestamp): number {
  const start = 1000; // Timestamps in the sample files start at 1000 seconds
  const setLength = 3; // 3 messages per set in the sample file

  return Math.floor((timestamp.seconds - start) / setLength);
}

/** Wait for a tick of the event loop */
export function tick() {
  return new Promise((resolve) => setImmediate(resolve));
}
