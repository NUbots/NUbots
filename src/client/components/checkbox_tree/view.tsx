import * as classnames from 'classnames'
import { observer } from 'mobx-react'
import * as React from 'react'
import { StatelessComponent } from 'react'

import { TreeModel } from './model'
import { TreeNodeModel } from './model'
import { TreeNode } from './tree_node/view'

export type CheckboxTreeProps = {
  model: TreeModel
  renderLabel?(node: TreeNodeModel): JSX.Element
  onCheck?(node: TreeNodeModel): void
  onExpand?(node: TreeNodeModel): void
}

export const CheckboxTree: StatelessComponent<CheckboxTreeProps> = (props: CheckboxTreeProps) => {
  return (
    <div>
      {
        props.model.nodes.map((node, i) => {
          return <TreeNode
            node={node}
            level={0}
            key={i}
            renderLabel={props.renderLabel}
            onCheck={props.onCheck}
            onExpand={props.onExpand}
          />
        })
      }
    </div>
  )
}
