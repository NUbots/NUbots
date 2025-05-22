/** Convert the given percentage to a timestamp in nanoseconds */
export function percentageToTimestamp(percentage: number, rangeStart: bigint, rangeEnd: bigint): bigint {
  return rangeStart + BigInt(Math.floor(percentage * Number(rangeEnd - rangeStart)));
}

/** Convert the given timestamp in nanoseconds to a percentage of the given range */
export function timestampToPercentage(timestamp: bigint, rangeStart: bigint, rangeEnd: bigint): number {
  return Number(timestamp - rangeStart) / Number(rangeEnd - rangeStart);
}
