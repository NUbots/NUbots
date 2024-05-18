import React from "react";
import { action } from "@storybook/addon-actions";
import { Meta, StoryObj } from "@storybook/react";
import { action as mobxAction, observable } from "mobx";
import { observer } from "mobx-react";

import { RobotModel } from "../../robot/model";
import { RobotSelectorSingle } from "../view";
import { themeDecorator } from "../../storybook_theme_decorator/view";

const meta: Meta<typeof RobotSelectorSingle> = {
  title: "components/RobotSelectorSingle",
  component: RobotSelectorSingle,
  decorators: [(story) => <div style={{ maxWidth: "320px" }}>{story()}</div>, themeDecorator],
};

export default meta;

type Story = StoryObj<typeof RobotSelectorSingle>;

const actions = {
  onSelect: action("onSelect"),
};

export const Empty: Story = {
  name: "empty",
  render: () => {
    return <RobotSelectorSingle robots={[]} onSelect={actions.onSelect} />;
  },
};

export const WithRobots: Story = {
  name: "with robots",
  render: () => {
    const robots = getRobots();
    return <RobotSelectorSingle robots={robots} onSelect={actions.onSelect} />;
  },
};

export const WithSelection: Story = {
  name: "with selection",
  render: () => {
    const robots = getRobots();
    const selected = robots[0];
    return <RobotSelectorSingle robots={robots} selected={selected} onSelect={actions.onSelect} />;
  },
};

export const Interactive: Story = {
  name: "interactive",
  render: () => {
    const robots = getRobots();
    const model = observable({
      robots,
      selected: robots[1],
    });
    const onSelect = mobxAction((robot: RobotModel) => (model.selected = robot));
    const Component = observer(() => (
      <RobotSelectorSingle robots={model.robots} selected={model.selected} onSelect={onSelect} />
    ));
    return <Component />;
  },
};

function getRobots(): RobotModel[] {
  return [
    {
      id: "1",
      name: "Virtual Robot 1",
      connected: true,
      enabled: true,
      address: "",
      port: 0,
    },
    {
      id: "2",
      name: "Virtual Robot 2",
      connected: true,
      enabled: true,
      address: "",
      port: 0,
    },
    {
      id: "3",
      name: "Virtual Robot 3",
      connected: true,
      enabled: true,
      address: "",
      port: 0,
    },
  ];
}
