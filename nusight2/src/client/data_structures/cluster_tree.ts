//
// Binary Tree with the constraint that the gap between its immediate
// children must be larger than the gaps between their children.
//
// This allows the content of the tree to be quickly retrieved at
// any 'detail level' where clusters of leaves with a gap smaller
// than the desired value are merged together into a cluster.
//
//                        o
//                   /         \
//                o               \
//             /     \               \
//           o         \               \
//         /   \         \               \
//        o     \           o              \
//       / \     \         / \              \
//      o   o     o       o   o              o
//
// Gap:  -3- --5-- ---7--- -3- ------14------
//

/**
 * Return value from a tree search representing a single data point.
 *
 * This contains a reference to a data point inserted in the tree.
 */
export interface TreeDataSingle<T> {
  type: "single";
  value: T;
}

/** Return value from a tree search representing a cluster of data points */
export interface TreeDataCluster<T> {
  type: "cluster";

  /** The highest data point of this cluster */
  max: T;

  /** The lowest data point of this cluster */
  min: T;

  /** The total number of data points in this cluster */
  childCount: number;
}

/**
 * A single return value for a tree search. This can be either a single data point or
 * a cluster of data points.
 */
export type TreeData<T> = TreeDataSingle<T> | TreeDataCluster<T>;

interface LeafNode<T, V extends number | bigint> {
  type: "leaf";

  /** The parent of this node. If undefined then this is the root. */
  parent?: InnerNode<T, V>;

  /** This node type points to an individual data point */
  data: TreeDataSingle<T>;
}

/** A node in the tree that is not a leaf */
interface InnerNode<T, V extends number | bigint> {
  type: "inner";

  /** The child of this node with lower data points.  */
  low: MaybeLazyNode<T, V>;

  /** The child of this node with higher data points.  */
  high: MaybeLazyNode<T, V>;

  /**
   * The high side of the gap between this node's children.
   * This is the lowest data point of the high side.
   */
  gapMax: V;

  /**
   * The low side of the gap between this node's children.
   * This is the highest data point of the low side
   */
  gapMin: V;

  /** The size of the gap between this node's children. This is `gapMax - gapMin`. */
  gapSize: V;

  /** This node type points to a cluster of data points. */
  data: TreeDataCluster<T>;
}

/** A node that hasn't been computed yet. Stores the range that it represents. */
interface LazyNode<T, V extends number | bigint> {
  type: "lazy";
  firstIndex: number;
  lastIndex: number;
  /** Will be called once this lazy node is resolved */
  onResolved: (resolvedNode: Node<T, V>) => void;
}

type Node<T, V extends number | bigint> = InnerNode<T, V> | LeafNode<T, V>;

type MaybeLazyNode<T, V extends number | bigint> = Node<T, V> | LazyNode<T, V>;

/**
 * Data structure for clustering values of a data series together.
 *
 * When the objects stored in this data structure are retrieved, a minimum `gap` can be
 * specified, where any elements that are closer together than this gap size are
 * grouped together into clusters.
 *
 * This allows for efficient viewing of large data series when the series only needs to
 * be viewed at a lower resolution.
 */
export class ClusteringTree<T, V extends number | bigint> {
  private root?: Node<T, V>;

  private data: T[] = [];

  /**
   * Each element in this array stores the gap between the corresponding element in the data array
   * and the one before it. For the first element, the gap is -1.
   */
  private gapSizes: number[] = [];

  /**
   * Constructor takes a list of starting data of the tree's type and a callback that takes
   * an item in the tree and returns its numeric value, which is used for sorting and clustering.
   *
   * The data must already be sorted by its value.
   */
  constructor(
    private readonly getValue: (value: T) => V,
    data?: T[],
  ) {
    if (!data) {
      return;
    }

    // Empty array, no root
    if (data.length === 0) {
      return;
    }

    this.setData(data);
  }

  /** Set the data stored in this tree */
  setData(data: T[]) {
    this.data = data;
    this.gapSizes = data.map((d, i) => (i === 0 ? -1 : this.getValue(d) - this.getValue(data[i - 1])));
    this.resolveLazyNode({
      type: "lazy",
      firstIndex: 0,
      lastIndex: data.length - 1,
      onResolved: (node) => (this.root = node),
    });
  }

  /**
   * Find the index of the largest gap between nodes in the specified range. This is used to build the
   * tree, where the largest gap between a node's children determines where to split the
   * child branches of the node.
   */
  private largestGapIndex(minIndex: number, maxIndex: number) {
    let largestGap = -Infinity;
    let largestGapIndex = -1;

    for (let i = minIndex; i <= maxIndex; i++) {
      const gapSize = this.gapSizes[i];
      if (
        gapSize > largestGap ||
        // If the gap equals the current largest, give it a chance at being selected as the
        // largest gap. This results in a more balanced tree if many of the data points have
        // the same gap size, which makes the tree have more consistent access times.
        (gapSize === largestGap && Math.random() < 1 / (maxIndex - minIndex))
      ) {
        largestGap = this.gapSizes[i];
        largestGapIndex = i;
      }
    }

    return largestGapIndex;
  }

  /** Convert a potentially lazy node in the tree to a resolved node */
  private resolveLazyNode(node: MaybeLazyNode<T, V>): Node<T, V> {
    if (node.type !== "lazy") {
      return node;
    }

    // Only a single node, make a leaf
    if (node.firstIndex === node.lastIndex) {
      const resolvedNode: LeafNode<T, V> = {
        type: "leaf",
        data: {
          type: "single",
          value: this.data[node.firstIndex],
        },
      };
      node.onResolved(resolvedNode);
      return resolvedNode;
    }

    // Find the largest gap between this node's children
    const largestGapIdx = this.largestGapIndex(node.firstIndex, node.lastIndex);
    this.gapSizes[largestGapIdx] = -1;

    // Get the values of the nodes either side of the gap
    const gapMin = this.getValue(this.data[largestGapIdx - 1]);
    const gapMax = this.getValue(this.data[largestGapIdx]);

    // Create the inner node
    const newNode: InnerNode<T, V> = {
      type: "inner",
      gapMin,
      gapMax,
      gapSize: (gapMax - gapMin) as V,
      data: {
        type: "cluster",
        min: this.data[node.firstIndex],
        max: this.data[node.lastIndex],
        childCount: node.lastIndex - node.firstIndex + 1,
      },
      // Make the children lazy nodes, where their indices are set to
      // either side of the largest gap between all this node's children
      low: {
        type: "lazy",
        firstIndex: node.firstIndex,
        lastIndex: largestGapIdx - 1,
        onResolved: (node) => (newNode.low = node),
      },
      high: {
        type: "lazy",
        firstIndex: largestGapIdx,
        lastIndex: node.lastIndex,
        onResolved: (node) => (newNode.high = node),
      },
    };
    node.onResolved(newNode);
    return newNode;
  }

  /**
   * Search the tree using a search callback. The search callback returns either a boolean indicating
   * if the node being searched for is found, or the next node to continue the search at (typically a
   * child of the current node). Returns the node if found, otherwise `undefined`
   */
  private searchTree(searchFn: (node: Node<T, V>) => MaybeLazyNode<T, V> | boolean): Node<T, V> | undefined {
    if (!this.root) {
      return undefined;
    }

    let current = this.root;
    let result = searchFn(this.root);

    while (typeof result === "object") {
      current = this.resolveLazyNode(result);
      result = searchFn(current);
    }

    return result === true ? current : undefined;
  }

  /**
   * Return set of elements from the tree in the given range with a gap size greater than the given size.
   *
   * Allows the data to be retrieved at specific 'resolution', where any group nodes with a gap smaller than
   * the specified gap are merged into a cluster.
   */
  getData(gapSize: V, min: V, max: V): TreeData<T>[] {
    if (!this.root) {
      return [];
    }

    const nodes: TreeData<T>[] = [];
    const nodesToVisit: MaybeLazyNode<T, V>[] = [this.root];

    // Depth first search through the tree culling nodes outside of the requested
    // range and only searching down to the requested gap size
    while (nodesToVisit.length > 0) {
      // Visit the next node
      const node = this.resolveLazyNode(nodesToVisit.pop()!);

      // If the node is an inner node...
      if (node.type === "inner") {
        // Ignore it if it is outside the requested range
        if (this.getValue(node.data.max) < min || this.getValue(node.data.min) > max) {
          continue;
        }

        // Add it to the results if its gap size is smaller than the target, otherwise continue
        // searching its children
        if (node.gapSize < gapSize) {
          nodes.push(node.data);
        } else {
          nodesToVisit.push(node.high, node.low);
        }
      }
      // If the node is a leaf and it is in view, add it to the results
      else if (this.getValue(node.data.value) > min && this.getValue(node.data.value) < max) {
        nodes.push(node.data);
      }
    }

    return nodes;
  }

  /**
   * Returns a pair of values containing the first value or cluster below a specified value
   * and the first value or cluster above a specified value.
   *
   * If no value is given for the gap size, the returned values will always be single data points.
   */
  getBoundingData(min: V, max: V): [TreeDataSingle<T> | undefined, TreeDataSingle<T> | undefined];
  getBoundingData(min: V, max: V, gapSize: V): [TreeData<T> | undefined, TreeData<T> | undefined];
  getBoundingData(min: V, max: V, gapSize?: V): [TreeData<T> | undefined, TreeData<T> | undefined] {
    const firstNodeBelow = this.searchTree((node) => {
      const dataPoint = node.type === "leaf" ? node.data.value : node.data.max;

      // If reached a leaf or the gap size is smaller than the target, return the current node if it
      // is below the min value
      if (node.type === "leaf" || (gapSize && node.gapSize < gapSize)) {
        return this.getValue(dataPoint) < min;
      }

      // Go to the larger child node that is less than the min
      return node.gapMax < min ? node.high : node.low;
    });

    const firstNodeAbove = this.searchTree((node) => {
      const dataPoint = node.type === "leaf" ? node.data.value : node.data.min;

      // If reached a leaf or the gap size is smaller than the target, return the current node if it
      // is above the max value
      if (node.type === "leaf" || (gapSize && node.gapSize < gapSize)) {
        return this.getValue(dataPoint) > max;
      }

      // Go to the smaller child node that is greater than the max
      return node.gapMin > max ? node.low : node.high;
    });

    return [firstNodeBelow?.data, firstNodeAbove?.data];
  }

  /** Get the index of a value in the tree. If the value is not found, returns -1 */
  indexOf(value: T): number {
    let idx = 0;
    const found = this.searchTree((node) => {
      if (node.type === "leaf") {
        return node.data.value === value;
      }

      const isHighSide = this.getValue(value) > node.gapMin;
      const lowSideChildren = node.low.type === "inner" ? node.low.data.childCount : 1;
      idx += isHighSide ? lowSideChildren : 0;
      return isHighSide ? node.high : node.low;
    });

    return found ? idx : -1;
  }
}
