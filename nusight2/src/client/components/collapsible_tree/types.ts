export interface RootNode<T = {}> {
  children: Node<T>[];
}

/**
 * A node in a collapsible tree.
 * The generic allows extra attributes to be added to the node.
 */
export type Node<T = {}> = InternalNode<T> | LeafNode<T>;

export type InternalNode<T = {}> = T & {
  type: "internal";
  name: string;
  isExpanded: boolean;
  children: Node<T>[];
};

export type LeafNode<T = {}> = T & {
  type: "leaf";
  name: string;
};

/**
 * Props for a node label in a collapsible tree.
 * The generic determines the type of node used.
 */
export interface NodeLabelProps<T = Node> {
  node: T;
  path: string;
  level: number;
  toggleButton: React.ReactNode;
}

/**
 * Props for the children of a node in a collapsible tree.
 * The generic determines the type of node used.
 */
export interface NodeContentProps<T = Node> {
  node: T;
  path: string;
  level: number;
  children: () => React.ReactNode;
}

/** Component to render a label for a node in a collapsible tree */
export type NodeLabelComponentType<T = {}> = React.ComponentType<NodeLabelProps<Node<T>>>;

/** Component to render the children for a node in a collapsible tree */
export type NodeContentComponentType<T = {}> = React.ComponentType<NodeContentProps<Node<T>>>;
