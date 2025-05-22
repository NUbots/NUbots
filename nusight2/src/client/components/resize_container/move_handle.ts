import { normaliseValues } from "./normalise_values";

/**
 * Calculates the new sizes of a set of children when a specified child's size is adjusted by an amount.
 *
 * Shrinks the children in the direction of movement and expands the first child in the opposite direction,
 * while applying the constraints that the children cannot shrink below their minimum size, and the total
 * combined size of the children cannot change.
 *
 * Returns the new ratios of the children and a boolean indicating if the limit has been reached for how
 * far the handle can be moved.
 */
export function moveHandle(
  index: number,
  sizeChange: number,
  containerSize: number,
  childRatios: number[],
  minSizes: ReadonlyArray<number>,
  maxSizes: ReadonlyArray<number>,
): {
  newRatios: number[];
  atLimit: boolean;
} {
  // If moving in negative direction, reverse the data and the target index to make
  // the children shrink in the other direction
  if (sizeChange < 0) {
    childRatios = childRatios.slice().reverse();
    minSizes = minSizes.slice().reverse();
    maxSizes = maxSizes.slice().reverse();
    index = childRatios.length - 2 - index;
  }

  // Normalise container ratios
  const newRatios = normaliseValues(childRatios);

  // Calculate the amount that each section can shrink by before hitting its minimum size
  const shrinkableSpace = newRatios.map((ratio, i) => ratio - minSizes[i] / containerSize, 0);

  // Calculate the space that each section can expand before reaching its maximum size
  const expandableSpace = newRatios.map((ratio, i) => maxSizes[i] / containerSize - ratio, 0);

  // Keep track of the total amount that the growing elements can expand before all reaching their maximum
  const totalExpandableSpace = expandableSpace.slice(0, index + 1).reduce((prev, curr) => prev + curr, 0);

  // Keep track of how much space is left to distribute and how much has been distributed.
  // The space to distribute is limited by the amount that the growing elements are allowed to grow.
  let spaceToDistribute = Math.min(Math.abs(sizeChange), totalExpandableSpace);
  let distributedSpace = 0;

  // Starting at the section after the target, shrink the section as much as possible until the space
  // to distribute reaches zero or it hits its minimum size. After this, continue to the next section.
  for (let i = index + 1; i < newRatios.length; i++) {
    const removedRatio = Math.min(shrinkableSpace[i], spaceToDistribute);
    spaceToDistribute -= removedRatio;
    distributedSpace += removedRatio;
    newRatios[i] = newRatios[i] - removedRatio;
  }

  // Starting at the target and moving backwards, distribute the space to the elements without letting
  // them grow above their maximum size
  for (let i = index; i >= 0; i--) {
    const addedRatio = Math.min(expandableSpace[i], distributedSpace);
    distributedSpace -= addedRatio;
    newRatios[i] = newRatios[i] + addedRatio;
  }

  // Reverse the children again to get them back to the original order
  if (sizeChange < 0) {
    newRatios.reverse();
  }

  return { newRatios, atLimit: spaceToDistribute > 0 || totalExpandableSpace < Math.abs(sizeChange) };
}
