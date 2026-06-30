import { ScrubberState, ScrubberState_StateEnum } from "@proto/message/eye/Scrubber";
import { NbsPacket, NbsTypeSubtypeBuffer } from "nbsdecoder.js";

import { hashType } from "../../shared/nuclearnet/hash_type";
import { Timestamp } from "../../shared/time/timestamp";

import { Scrubber } from "./scrubber";
import { nanosToTimestampObject } from "./utils";

interface SyntheticType {
  event: string;
  type: Buffer;
  subtype: number;
}

/** The types we can synthesize. Maps type hash string to type details. */
const types: Map<string, SyntheticType> = new Map();

// Add the ScrubberState type
types.set(hashType("message.eye.ScrubberState").toString("hex"), {
  event: "message.eye.ScrubberState",
  type: hashType("message.eye.ScrubberState"),
  subtype: 0,
});

/** Get a list of all the types we can synthesize */
export function availableSyntheticTypes(): NbsTypeSubtypeBuffer[] {
  return Array.from(types.values());
}

/** Check if the given type can be synthesized */
export function isSynthesizable(type: NbsTypeSubtypeBuffer) {
  return types.has(type.type.toString("hex"));
}

const ScrubberPlaybackStateToEnum = {
  paused: ScrubberState_StateEnum.PAUSED,
  playing: ScrubberState_StateEnum.PLAYING,
  ended: ScrubberState_StateEnum.ENDED,
  unknown: ScrubberState_StateEnum.UNKNOWN,
} as const;

/** Create an NBS packet of the given synthesizable type */
export function synthesize(typeSubtype: NbsTypeSubtypeBuffer, scrubber: Scrubber, timestampNanos: bigint): NbsPacket {
  const type = types.get(typeSubtype.type.toString("hex"));

  if (!type) {
    throw new Error(`Type ${typeSubtype.type.toString("hex")} is not a known synthesizable type`);
  }

  const timestamp = nanosToTimestampObject(timestampNanos);

  switch (type.event) {
    case "message.eye.ScrubberState": {
      const scrubberState = new ScrubberState({
        timestamp: Timestamp.toMessage(timestamp),
        id: scrubber.data.id,
        name: scrubber.data.name,
        start: Timestamp.toMessage(scrubber.data.start),
        end: Timestamp.toMessage(scrubber.data.end),
        playbackState: ScrubberPlaybackStateToEnum[scrubber.data.playbackState],
        playbackSpeed: scrubber.data.playbackSpeed,
        playbackRepeat: scrubber.data.playbackRepeat,
      }).toBinary();

      return {
        timestamp,
        ...typeSubtype,
        payload: Buffer.from(scrubberState),
      };
    }

    default:
      throw new Error(`Event ${type.event} does not match a known synthesizable type`);
  }
}
