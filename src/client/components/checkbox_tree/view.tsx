import React from 'react'
import { StatelessComponent } from 'react'

import { TreeModel } from './model'
import { TreeNodeModel } from './model'
import { TreeNode } from './tree_node/view'

export type CheckboxTreeProps = {
  model: TreeModel
  renderLabel?(node: TreeNodeModel): JSX.Element | string
  onCheck?(node: TreeNodeModel): void
  onExpand?(node: TreeNodeModel): void
}

export const CheckboxTree: StatelessComponent<CheckboxTreeProps> = (props: CheckboxTreeProps) => {
  return (
    <div>
      {props.model.nodes.map((node, i) => (
        <TreeNode
          key={i}
          node={node}
          level={0}
          renderLabel={props.renderLabel}
          onCheck={props.onCheck}
          onExpand={props.onExpand}
        />
      ))}
    </div>
  )
}
