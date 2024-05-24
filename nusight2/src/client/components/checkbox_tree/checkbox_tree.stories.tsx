import React from "react";
import { Meta, StoryObj } from "@storybook/react";
import { action, computed, observable } from "mobx";
import { createTransformer } from "mobx-utils";

import { ThemeDecorator } from "../storybook_theme_decorator/view";

import { CheckedState, TreeModel, TreeNodeModel } from "./model";
import { CheckboxTree } from "./view";

const meta: Meta<typeof CheckboxTree> = {
  title: "components/CheckboxTree",
  component: CheckboxTree,
  decorators: [ThemeDecorator],
};

export default meta;

type Story = StoryObj<typeof CheckboxTree>;

class TreeData {
  @observable checked: CheckedState;
  // This is where data for the tree node would be stored, e.g. on a `value` property.
  // Omitted here as it's not necessary for demonstrating the CheckboxTree component.

  constructor({ checked }: { checked?: CheckedState } = {}) {
    this.checked = checked ?? CheckedState.Checked;
  }
}

type ToggleTreeChildren = { [key: string]: TreeData | ToggleTreeChildren };

class ToggleTree {
  @observable model: ToggleTreeChildren | TreeData;
  @observable label: string;
  @observable expanded: boolean;

  constructor(opts: { label: string; model: ToggleTreeChildren | TreeData }) {
    this.model = opts.model;
    this.label = opts.label;
    this.expanded = false;
  }

  static of = createTransformer((opts: { label: string; model: ToggleTreeChildren | TreeData }): ToggleTree => {
    return new ToggleTree(opts);
  });

  @computed
  get leaf(): boolean {
    return this.model instanceof TreeData;
  }

  @computed
  get checked(): CheckedState {
    if (this.model instanceof TreeData) {
      return this.model.checked;
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
    if (this.model instanceof TreeData) {
      this.model.checked = checked;
    } else {
      this.children.forEach((child) => {
        child.checked = checked;
      });
    }
  }

  @computed
  get children(): TreeNodeModel[] {
    if (this.model instanceof TreeData) {
      return [];
    }

    return Array.from(
      Object.entries(this.model).map(([key, value]) =>
        ToggleTree.of({
          label: key,
          model: value,
        }),
      ),
    );
  }
}

export const Default: Story = {
  name: "default",
  render: () => {
    const rawData: ToggleTreeChildren = {
      "Robot 1": {
        "Debug Waves": {
          "sin()": new TreeData(),
          "cos()": new TreeData(),
          "tan()": new TreeData(),
        },
        Position: {
          x: new TreeData(),
          y: new TreeData(),
          z: new TreeData(),
        },
      },
      "Robot 2": {
        "Debug Waves": {
          "sin()": new TreeData(),
          "cos()": new TreeData(),
          "tan()": new TreeData(),
        },
        Position: {
          x: new TreeData(),
          y: new TreeData(),
          z: new TreeData(),
        },
      },
    };

    const model = observable<TreeModel>({
      nodes: Object.entries(rawData).map(([key, value]) => {
        return ToggleTree.of({ label: key, model: value });
      }),
      usePessimisticToggle: true,
    });

    // Used to check/uncheck a node in the tree
    const onNodeCheck = action((node: TreeNodeModel) => {
      node.checked = node.checked === CheckedState.Checked ? CheckedState.Unchecked : CheckedState.Checked;
    });

    // Used to expand/collapse a node in the tree
    const onNodeExpand = action((node: TreeNodeModel) => {
      node.expanded = !node.expanded;
    });

    // Used to render the label, can be any arbitrary content, e.g. a color picker
    // similar to the one used in the Chart view's checkbox tree.
    const renderLabel = (node: TreeNodeModel) => {
      return <span>{node.label}</span>;
    };

    return (
      <div className="max-w-[320px]">
        <CheckboxTree model={model} onCheck={onNodeCheck} onExpand={onNodeExpand} renderLabel={renderLabel} />
      </div>
    );
  },
};
