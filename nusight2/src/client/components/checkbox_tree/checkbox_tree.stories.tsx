import React from 'react'
import { storiesOf } from '@storybook/react'
import { action, computed, observable } from 'mobx'
import { observer } from 'mobx-react'
import { createTransformer } from 'mobx-utils'

import { CheckedState, TreeModel, TreeNodeModel } from './model'

import { CheckboxTree } from './view'

class DataPoint {
  @observable value: number
  @observable checked: CheckedState

  constructor({ value, checked }: { value: number; checked?: CheckedState }) {
    this.value = value
    this.checked = checked ?? CheckedState.Checked
  }
}

type ToggleTreeChildren = { [key: string]: DataPoint | ToggleTreeChildren }

class ToggleTree {
  @observable model: ToggleTreeChildren | DataPoint
  @observable label: string
  @observable expanded: boolean

  constructor(opts: { label: string; model: ToggleTreeChildren | DataPoint }) {
    this.model = opts.model
    this.label = opts.label
    this.expanded = false
  }

  static of = createTransformer(
    (opts: { label: string; model: ToggleTreeChildren | DataPoint }): ToggleTree => {
      return new ToggleTree(opts)
    },
  )

  @computed
  get leaf(): boolean {
    return this.model instanceof DataPoint
  }

  @computed
  get checked(): CheckedState {
    if (this.model instanceof DataPoint) {
      return this.model.checked
    }

    if (this.children.every(node => node.checked === CheckedState.Checked)) {
      return CheckedState.Checked
    }

    if (this.children.every(node => node.checked === CheckedState.Unchecked)) {
      return CheckedState.Unchecked
    }

    return CheckedState.Indeterminate
  }

  set checked(checked: CheckedState) {
    if (this.model instanceof DataPoint) {
      this.model.checked = checked
    } else {
      this.children.forEach(child => {
        child.checked = checked
      })
    }
  }

  @computed
  get children(): TreeNodeModel[] {
    if (this.model instanceof DataPoint) {
      return []
    }

    return Array.from(
      Object.entries(this.model).map(([key, value]) =>
        ToggleTree.of({
          label: key,
          model: value,
        }),
      ),
    )
  }
}

storiesOf('components/CheckboxTree', module)
  .addDecorator(story => <div style={{ maxWidth: '320px' }}>{story()}</div>)
  .add('renders', () => {
    const rawData: ToggleTreeChildren = {
      'Robot 1': {
        'Debug Waves': {
          'sin()': new DataPoint({ value: 1 }),
          'cos()': new DataPoint({ value: 2 }),
          'tan()': new DataPoint({ value: 3 }),
        },
        Position: {
          x: new DataPoint({ value: 5 }),
          y: new DataPoint({ value: 6 }),
          z: new DataPoint({ value: 7 }),
        },
      },
      'Robot 2': {
        'Debug Waves': {
          'sin()': new DataPoint({ value: 8 }),
          'cos()': new DataPoint({ value: 9 }),
          'tan()': new DataPoint({ value: 10 }),
        },
        Position: {
          x: new DataPoint({ value: 11 }),
          y: new DataPoint({ value: 12 }),
          z: new DataPoint({ value: 13 }),
        },
      },
    }

    const model = observable<TreeModel>({
      nodes: Object.entries(rawData).map(([key, value]) => {
        return ToggleTree.of({ label: key, model: value })
      }),
      usePessimisticToggle: true,
    })

    // Used to check/uncheck a node in the tree
    const onNodeCheck = action((node: TreeNodeModel) => {
      node.checked =
        node.checked === CheckedState.Checked ? CheckedState.Unchecked : CheckedState.Checked
    })

    // Used to expand/collapse a node in the tree
    const onNodeExpand = action((node: TreeNodeModel) => {
      node.expanded = !node.expanded
    })

    // Used to render the label, can be any arbitrary content
    const renderLabel = (node: TreeNodeModel) => {
      return <span>{node.label}</span>
    }

    const Story = observer(() => (
      <CheckboxTree
        model={model}
        onCheck={onNodeCheck}
        onExpand={onNodeExpand}
        renderLabel={renderLabel}
      />
    ))

    return <Story />
  })
