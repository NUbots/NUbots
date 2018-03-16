import { computed } from 'mobx'
import { observable } from 'mobx'

import { memoize } from '../../base/memoize'

export enum CheckedState {
  Checked = 'checked',
  Unchecked = 'unchecked',
  Indeterminate = 'indeterminate',
}

export type TreeNodeModelOpts = {
  children: TreeNodeModel[]
  checked: CheckedState
  label: string
  expanded: boolean
}

export interface TreeNodeModel {
  children: TreeNodeModel[]
  checked: CheckedState
  label: string
  expanded: boolean
  leaf: boolean
}

export type TreeModelOpts = {
  nodes: TreeNodeModel[]
  usePessimisticToggle: boolean
}

export interface TreeModel {
  nodes: TreeNodeModel[]
  usePessimisticToggle: boolean
}
