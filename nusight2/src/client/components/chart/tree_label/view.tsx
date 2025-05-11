import React from "react";
import { Component } from "react";
import { observer } from "mobx-react";
import { ColorResult } from "react-color";
import { TwitterPicker } from "react-color";

import { TreeNodeModel } from "../../checkbox_tree/model";
import { TreeViewModel } from "../view_model";

type TreeLabelProps = {
  node: TreeNodeModel;
  onColorChange?(color: string, node: TreeNodeModel): void;
  onMouseEnter?(node: TreeNodeModel): void;
  onMouseLeave?(node: TreeNodeModel): void;
};

@observer
export class TreeLabel extends Component<TreeLabelProps> {
  state = {
    showColorPicker: false,
  };

  render() {
    const node = this.props.node;

    if (!(node instanceof TreeViewModel)) {
      throw new Error(`Unsupported node type: ${node}`);
    }

    if (!node.leaf) {
      return (
        <div className="flex items-center mr-4" onMouseEnter={this.onMouseEnter} onMouseLeave={this.onMouseLeave}>
          {node.label}
        </div>
      );
    }

    return (
      <div className="flex items-center mr-4" onMouseEnter={this.onMouseEnter} onMouseLeave={this.onMouseLeave}>
        <span className="flex-grow">{node.label}</span>

        <button
          className="w-3 h-3 rounded-full"
          onClick={this.togglePicker}
          style={{ backgroundColor: node.color }}
        ></button>

        {this.state.showColorPicker && (
          <div className="absolute z-10 right-2">
            <div className="fixed top-0 right-0 bottom-0 left-0" onClick={this.closePicker}></div>
            <TwitterPicker color={node.color} onChangeComplete={this.onColorChange} triangle="hide" />
          </div>
        )}
      </div>
    );
  }

  onMouseEnter = () => {
    if (this.props.onMouseEnter) {
      this.props.onMouseEnter(this.props.node);
    }
  };

  onMouseLeave = () => {
    if (this.props.onMouseLeave) {
      this.props.onMouseLeave(this.props.node);
    }
  };

  onColorChange = (color: ColorResult) => {
    if (this.props.onColorChange) {
      this.props.onColorChange(color.hex, this.props.node);
      this.closePicker();
    }
  };

  togglePicker = () => {
    this.setState({ showColorPicker: !this.state.showColorPicker });
  };

  closePicker = () => {
    this.setState({ showColorPicker: false });
  };
}
