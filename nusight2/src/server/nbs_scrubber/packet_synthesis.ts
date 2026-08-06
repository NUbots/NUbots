import {
  ScrubberIndex,
  ScrubberIndex_TypeIndex,
  ScrubberState,
  ScrubberState_StateEnum,
} from "@proto/message/eye/Scrubber";
import { NbsPacket, NbsTypeSubtypeBuffer } from "nbsdecoder.js";

import { messageHashToName } from "../../shared/messages/generated/hash_converters";
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

// Add the ScrubberIndex type
types.set(hashType("message.eye.ScrubberIndex").toString("hex"), {
  event: "message.eye.ScrubberIndex",
  type: hashType("message.eye.ScrubberIndex"),
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

/** The hash of the ScrubberIndex type, which is synthesized once and never changes */
const scrubberIndexTypeHash = hashType("message.eye.ScrubberIndex").toString("hex");

/**
 * Check if the given type is a static synthetic type: one that is synthesized once and
 * does not change over time, so it does not need to be re-emitted during playback.
 */
export function isStaticSyntheticType(type: NbsTypeSubtypeBuffer): boolean {
  return type.type.toString("hex") === scrubberIndexTypeHash;
}

/** Cache of the built scrubber index payload, keyed by scrubber */
const indexPayloadCache = new WeakMap<Scrubber, Buffer>();

/** Build, or get the cached, ScrubberIndex payload for the given scrubber */
function getIndexPayload(scrubber: Scrubber): Buffer {
  const cached = indexPayloadCache.get(scrubber);
  if (cached !== undefined) {
    return cached;
  }

  const getMessageName = (hash: Buffer | string) => {
    try {
      return messageHashToName(hash);
    } catch {
      // Empty string indicates that we don't know the message name
      return "";
    }
  };

  const indices = scrubber.decoder.getAvailableTypes().map((typeSubtype) => {
    const timestamps = scrubber.decoder.getTypeIndex(typeSubtype);
    return new ScrubberIndex_TypeIndex({
      typeName: getMessageName(typeSubtype.type),
      typeHash: typeSubtype.type.toString("hex"),
      subtype: typeSubtype.subtype,
      timestamps: timestamps as unknown as ScrubberIndex_TypeIndex["timestamps"],
    });
  });

  const payload = Buffer.from(new ScrubberIndex({ id: scrubber.data.id, indices }).toBinary());
  indexPayloadCache.set(scrubber, payload);
  return payload;
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

    case "message.eye.ScrubberIndex": {
      return {
        timestamp,
        ...typeSubtype,
        payload: getIndexPayload(scrubber),
      };
    }

    default:
      throw new Error(`Event ${type.event} does not match a known synthesizable type`);
  }
}
