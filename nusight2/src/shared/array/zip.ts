/** Zip two arrays together into an array of tuples. */
export function zip<A, B>(a: A[], b: B[]): [A, B][] {
  return a.map((_, i) => [a[i], b[i]]);
}
