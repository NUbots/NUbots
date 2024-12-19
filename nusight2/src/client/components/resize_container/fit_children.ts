import { normaliseValues } from "./normalise_values";

/**
 * Calculates the best fit for a set of children in a container using the target ratios between
 * the children's sizes and the minimum/maximum size they can be in pixels.
 *
 * Returns the new sizes as percentages of the container size.
 */
export function fitChildren(
  containerSize: number,
  childRatios: number[],
  minSizes: ReadonlyArray<number>,
  maxSizes: ReadonlyArray<number>,
): number[] {
  childRatios = normaliseValues(childRatios);

  // Find which children will be set to their limit
  const atMinimum = childRatios.map((ratio, i) => ratio * containerSize < minSizes[i]);
  const atMaximum = childRatios.map((ratio, i) => ratio * containerSize > maxSizes[i]);
  const atLimit = childRatios.map((_, i) => atMinimum[i] || atMaximum[i]);

  // If any hit their limit the remaining children are distributed over the remaining space
  if (atLimit.includes(true)) {
    // Calculate how much space the children at their size limit will occupy
    const minimumSum = minSizes.reduce((acc, min, i) => acc + (atMinimum[i] ? min / containerSize : 0), 0);
    const maximumSum = maxSizes.reduce((acc, max, i) => acc + (atMaximum[i] ? max / containerSize : 0), 0);

    // Remaining percent of container left after removing space taken by children at their limit
    const remainingPercent = 1.0 - (minimumSum + maximumSum);

    // Calculate next set of values to fit children not at their limit into the remaining space. Values for
    // elements at their limit are set to 0 so they are ignored in recursive calls
    const remainingMinSizes = minSizes.map((size, i) => (atLimit[i] ? 0 : size));
    const remainingMaxSizes = maxSizes.map((size, i) => (atLimit[i] ? 0 : size));
    const remainingChildRatios = childRatios.map((ratio, i) => (atLimit[i] ? 0 : ratio));

    // Recursively fit the remaining children into the remaining space
    const nextFit = fitChildren(
      remainingPercent * containerSize,
      remainingChildRatios,
      remainingMinSizes,
      remainingMaxSizes,
    );

    // Apply the new fitting to the remaining children and set the rest to their limit
    return nextFit.map((ratio, i) =>
      atMinimum[i]
        ? minSizes[i] / containerSize
        : atMaximum[i]
          ? maxSizes[i] / containerSize
          : ratio * remainingPercent,
    );
  }

  return childRatios;
}
