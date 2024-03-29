import { Component } from "react";
import { MouseEvent } from "react";
import React from "react";
import classNames from "classnames";
import { autorun } from "mobx";
import { IReactionDisposer } from "mobx";
import { observer } from "mobx-react";

import { CheckedState } from "../model";
import { TreeNodeModel } from "../model";

import IconChevronDown from "./icon_chevron_down";
import IconChevronRight from "./icon_chevron_right";
import style from "./style.module.css";

export interface TreeNodeProps {
  node: TreeNodeModel;
  level?: number;
  renderLabel?(node: TreeNodeModel): JSX.Element | string;
  onCheck?(node: TreeNodeModel): void;
  onExpand?(node: TreeNodeModel): void;
  onMouseEnter?(node: TreeNodeModel): void;
  onMouseLeave?(node: TreeNodeModel): void;
}

@observer
export class TreeNode extends Component<TreeNodeProps> {
  private checkbox?: HTMLInputElement;
  private stopAutorun?: IReactionDisposer;

  componentDidMount() {
    if (!this.checkbox) {
      return;
    }

    // autorun is needed to update the indeterminate property of the checkbox, since the
    // HTML input element doesn't have an indeterminate attribute we can set in JSX.
    this.stopAutorun = autorun(() => this.updateCheckbox());
  }

  componentWillUnmount() {
    if (this.stopAutorun) {
      this.stopAutorun();
    }
  }

  render(): JSX.Element {
    const children = this.props.node.children;
    const hasChildren = children.length > 0;
    const level = this.props.level || 0;
    const classes = classNames(style.treenode);
    const renderLabel = this.props.renderLabel;

    // Using inline paddingLeft to indent so that the hover and selected background indicators
    // are full width. Padding is the default left padding of 8px plus each level's indent of 22px.
    const headerInlineStyle = {
      paddingLeft: 8 + level * 22 + "px",
    };

    return (
      <ul className={classes}>
        <li>
          <div
            className={style.treenode__header}
            style={headerInlineStyle}
            onClick={this.props.onExpand ? this.onClick : undefined}
            onMouseEnter={this.props.onMouseEnter ? this.onMouseEnter : undefined}
            onMouseLeave={this.props.onMouseLeave ? this.onMouseLeave : undefined}
          >
            <div className={style.treenode__icon}>
              {hasChildren ? this.props.node.expanded ? <IconChevronDown /> : <IconChevronRight /> : null}
            </div>

            <div className={style.treenode__checkbox}>
              <input type="checkbox" ref={this.onRef} onClick={this.onCheckboxClick} onChange={this.onCheckboxChange} />
            </div>

            <div className={style.treenode__label}>
              {renderLabel ? renderLabel(this.props.node) : this.props.node.label}
            </div>
          </div>

          {this.props.node.expanded &&
            children.map((node, i) => (
              <TreeNode
                key={i}
                node={node}
                level={level + 1}
                renderLabel={this.props.renderLabel}
                onCheck={this.props.onCheck}
                onExpand={this.props.onExpand}
                onMouseEnter={this.props.onMouseEnter}
                onMouseLeave={this.props.onMouseLeave}
              />
            ))}
        </li>
      </ul>
    );
  }

  private onRef = (checkbox: HTMLInputElement) => {
    this.checkbox = checkbox;
  };

  private onClick = () => {
    this.props.onExpand?.(this.props.node);
  };

  private onMouseEnter = () => {
    this.props.onMouseEnter?.(this.props.node);
  };

  private onMouseLeave = () => {
    this.props.onMouseLeave?.(this.props.node);
  };

  private updateCheckbox = () => {
    if (this.props.node.checked === CheckedState.Checked) {
      this.checkbox!.indeterminate = false;
      this.checkbox!.checked = true;
    } else if (this.props.node.checked === CheckedState.Unchecked) {
      this.checkbox!.indeterminate = false;
      this.checkbox!.checked = false;
    } else {
      this.checkbox!.indeterminate = true;
    }
  };

  private onCheckboxClick = (event: MouseEvent<HTMLInputElement>) => {
    event.stopPropagation();
  };

  private onCheckboxChange = () => {
    if (this.props.onCheck) {
      this.props.onCheck(this.props.node);
    }
  };
}
