import { getObjectFit } from "./object_fit";

/**
 * Given a container of a set size and a number of items, find the number of columns that will best fit
 * the items in the available space, taking into account the aspect ratio of the items.
 */
export function bestFitColumns(opts: {
  container: { width: number; height: number };
  itemAspectRatio: number;
  numItems: number;
}) {
  const { container, itemAspectRatio, numItems } = opts;
  const arr = [];
  for (let cols = 1; cols <= numItems; cols++) {
    const rows = Math.ceil(numItems / cols);
    const { width, height } = getObjectFit(
      {
        width: container.width / cols,
        height: container.height / rows,
      },
      {
        type: "contain",
        aspect: itemAspectRatio,
      },
    );
    arr.push({
      cols,
      coverage: (width * height * numItems) / (container.height * container.width),
    });
  }
  arr.sort((a, b) => b.coverage - a.coverage);
  return arr[0].cols;
}
