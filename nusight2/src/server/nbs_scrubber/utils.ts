import { NbsTimestamp } from "nbsdecoder.js";

import { Timestamp } from "../../shared/time/timestamp";

/** Convert the given NBS timestamp object to a BigInt of nanoseconds */
export function timestampObjectToNanos(timestamp: NbsTimestamp): bigint {
  return Timestamp.toNanos(timestamp);
}

/** Convert the given nanoseconds to an NBS timestamp object */
export function nanosToTimestampObject(nanos: bigint): NbsTimestamp {
  const ts = new Timestamp(nanos);
  return { seconds: ts.seconds, nanos: ts.nanos };
}
