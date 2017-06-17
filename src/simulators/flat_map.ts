/**
 * Run fn on each item in the given array, flatten the list of lists into a single list.
 *
 * e.g. flatMap(x => [x, x * x], [1, 2, 3]); // [1, 1, 2, 4, 3, 9]
 */
export function flatMap<T, U>(fn: (item: T) => U[], arr: T[]): U[] {
  return Array.prototype.concat.apply([], arr.map(fn))
}
