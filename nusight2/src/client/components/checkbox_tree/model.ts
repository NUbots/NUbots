export enum CheckedState {
  Checked = "checked",
  Unchecked = "unchecked",
  Indeterminate = "indeterminate",
}

export interface TreeNodeModel {
  /** Ordered array of children to render for this node in the checkbox tree */
  children: TreeNodeModel[];
  /** The checked state of this node: checked, unchecked, or indeterminate */
  checked: CheckedState;
  /** The label of this node to show in the checkbox tree */
  label: string;
  /** Whether or not this node is expanded in the checkbox tree */
  expanded: boolean;
  /** Whether or not this node is a leaf (i.e. it doesn't and will not have any children) */
  leaf: boolean;
}

export interface TreeModel {
  nodes: TreeNodeModel[];
  usePessimisticToggle: boolean;
}
