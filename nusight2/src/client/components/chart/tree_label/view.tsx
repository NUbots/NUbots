import { observer } from 'mobx-react'
import React from 'react'
import { Component } from 'react'
import { ColorResult } from 'react-color'
import { TwitterPicker } from 'react-color'

import { TreeNodeModel } from '../../checkbox_tree/model'
import { TreeViewModel } from '../view_model'

import style from './style.css'

type TreeLabelProps = {
  node: TreeNodeModel
  onColorChange?(color: string, node: TreeNodeModel): void
  onMouseEnter?(node: TreeNodeModel): void
  onMouseLeave?(node: TreeNodeModel): void
}

@observer
export class TreeLabel extends Component<TreeLabelProps> {
  state = {
    showColorPicker: false,
  }

  render() {
    const node = this.props.node

    if (!(node instanceof TreeViewModel)) {
      throw new Error(`Unsupported node type: ${node}`)
    }

    if (!node.leaf) {
      return (
        <div
          className={style.label}
          onMouseEnter={this.onMouseEnter}
          onMouseLeave={this.onMouseLeave}
        >
          {node.label}
        </div>
      )
    }

    return (
      <div
        className={style.label}
        onMouseEnter={this.onMouseEnter}
        onMouseLeave={this.onMouseLeave}
      >
        <span className={style.labelName}>{node.label}</span>

        <button
          className={style.pickerButton}
          onClick={this.togglePicker}
          style={{ backgroundColor: node.color }}
        ></button>

        {this.state.showColorPicker && (
          <div className={style.pickerPopover}>
            <div className={style.pickerPopoverCover} onClick={this.closePicker}></div>
            <TwitterPicker
              color={node.color}
              onChangeComplete={this.onColorChange}
              triangle="hide"
            />
          </div>
        )}
      </div>
    )
  }

  onMouseEnter = () => {
    if (this.props.onMouseEnter) {
      this.props.onMouseEnter(this.props.node)
    }
  }

  onMouseLeave = () => {
    if (this.props.onMouseLeave) {
      this.props.onMouseLeave(this.props.node)
    }
  }

  onColorChange = (color: ColorResult) => {
    if (this.props.onColorChange) {
      this.props.onColorChange(color.hex, this.props.node)
      this.closePicker()
    }
  }

  togglePicker = () => {
    this.setState({ showColorPicker: !this.state.showColorPicker })
  }

  closePicker = () => {
    this.setState({ showColorPicker: false })
  }
}
