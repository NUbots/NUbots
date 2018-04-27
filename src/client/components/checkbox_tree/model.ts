export enum CheckedState {
  Checked = 'checked',
  Unchecked = 'unchecked',
  Indeterminate = 'indeterminate',
}

export interface TreeNodeModel {
  children: TreeNodeModel[]
  checked: CheckedState
  label: string
  expanded: boolean
  leaf: boolean
}

export interface TreeModel {
  nodes: TreeNodeModel[]
  usePessimisticToggle: boolean
}
