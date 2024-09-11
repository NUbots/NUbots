import { computed } from "mobx";
import { observable } from "mobx";
import { createTransformer } from "mobx-utils";

import { CheckedState } from "../checkbox_tree/model";
import { TreeNodeModel } from "../checkbox_tree/model";

import { TreeData } from "./model";

interface TreeViewModelOpts {
  model: TreeData;
  label: string;
}

export class TreeViewModel implements TreeNodeModel {
  private model: TreeData;
  @observable label: string;
  @observable expanded: boolean;

  constructor(opts: TreeViewModelOpts) {
    this.model = opts.model;
    this.label = opts.label;
    this.expanded = false;
  }

  static of = createTransformer((opts: TreeViewModelOpts): TreeViewModel => {
    return new TreeViewModel(opts);
  });

  @computed
  get color(): string {
    // Non-leaf nodes have no color
    if (!this.leaf) {
      return "";
    }

    return (this.model as DataSeries).color;
  }

  set color(color: string) {
    if (!this.leaf) {
      throw new Error("Cannot set the color of a non-leaf node");
    }

    (this.model as DataSeries).color = color;
  }

  @computed
  get highlight(): boolean {
    // Non-leaf nodes have no color
    if (!this.leaf) {
      return false;
    }

    return (this.model as DataSeries).highlight;
  }

  set highlight(highlight: boolean) {
    if (!this.leaf) {
      throw new Error("Cannot set the highlight of a non-leaf node");
    }

    (this.model as DataSeries).highlight = highlight;
  }

  @computed
  get leaf(): boolean {
    return this.model instanceof DataSeries;
  }

  @computed
  get checked(): CheckedState {
    if (this.leaf) {
      return (this.model as DataSeries).checked;
    }

    if (this.children.every((node) => node.checked === CheckedState.Checked)) {
      return CheckedState.Checked;
    }

    if (this.children.every((node) => node.checked === CheckedState.Unchecked)) {
      return CheckedState.Unchecked;
    }

    return CheckedState.Indeterminate;
  }

  set checked(checked: CheckedState) {
    if (this.leaf) {
      (this.model as DataSeries).checked = checked;
    } else {
      this.children.forEach((child) => {
        child.checked = checked;
      });
    }
  }

  @computed
  get children(): TreeNodeModel[] {
    if (this.model instanceof DataSeries) {
      return [];
    }

    return Array.from(this.model.entries()).map((entry: [string, TreeData]) =>
      TreeViewModel.of({
        label: entry[0],
        model: entry[1],
      }),
    );
  }
}
