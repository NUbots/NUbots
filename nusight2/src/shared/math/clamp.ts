/**
 * Clamp the given value to be between the given range.
 *
 * @param value Value to clamp
 * @param min   Minimum value of result
 * @param max   Maximum value of result
 */
export function clamp(value: number, min: number, max: number): number;
export function clamp(value: bigint, min: bigint, max: bigint): bigint;
export function clamp<T = number | bigint>(value: T, min: T, max: T): T {
  return value < min ? min : value > max ? max : value;
}
