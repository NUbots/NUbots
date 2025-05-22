/**
 * Takes an array of values and returns a new set of values with the same ratios
 * but they now sum to 1.
 */
export function normaliseValues(values: number[]) {
  const sum = values.reduce((prev, curr) => prev + curr, 0);

  // Replace infinities with 1 to prevent NaN values. Infinities are possible here because
  // they are used to represent the pixel size of maximised containers
  return values.map((value) => (value === Infinity ? 1 : value / sum));
}
